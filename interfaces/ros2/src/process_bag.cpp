#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros2/publisher.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace {
template <typename T>
T deserialize(const rclcpp::SerializedMessage& serialized) {
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &msg);
  return msg;
}

// Optional "--max_scans N" flag: stop reading after N point clouds. The bag is
// then closed and the map saved through the normal path, so a short smoke run
// still exercises the trajectory/map export. 0 (default) = read the whole bag.
size_t parseMaxScans(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--max_scans") {
      return static_cast<size_t>(std::stoull(argv[i + 1]));
    }
  }
  return 0;
}

// Optional "--start_offset_s S": drop every message in the first S seconds of
// the bag. Combined with map.initial_pose this starts a localization run in the
// middle of a frozen map. 
double parseStartOffset(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--start_offset_s") {
      return std::stod(argv[i + 1]);
    }
  }
  return 0.0;
}

// Optional "--rate R": throttle replay to R times real time (1.0 = wall clock),
// which is what makes an RViz session watchable. 0 (default) = as fast as the
// hardware allows, the right setting for batch runs.
double parseRate(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--rate") {
      return std::stod(argv[i + 1]);
    }
  }
  return 0.0;
}
}  // namespace

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bievr_lio_bag_node");

  bievr::Config config;
  // rclcpp::init does not strip ROS arguments from argv; remove_ros_arguments
  // returns just the application arguments (our config-file flags).
  if (!bievr::loadConfigFromArgs(rclcpp::remove_ros_arguments(argc, argv), config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  auto pipeline = std::make_shared<bievr::Pipeline>(config.pipeline_config);
  auto synchronizer = std::make_shared<bievr::Synchronizer>(pipeline);
  auto lio_pub = std::make_shared<bievr::Publisher>(node, pipeline, "bievr_lio");

  rosbag2_cpp::Reader reader;
  reader.open(config.topic_config.bag_path);

  // Map each topic to its message type (needed to pick the right deserializer).
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  const std::string& imu_topic = config.topic_config.imu_topic;

  const size_t max_scans = parseMaxScans(argc, argv);
  size_t n_scans = 0;
  LOG(I, max_scans > 0, "Stopping after " << max_scans << " point clouds (--max_scans).");

  const double start_offset_s = parseStartOffset(argc, argv);
  int64_t t_first_ns = -1;
  LOG(I, start_offset_s > 0.0,
      "Skipping the first " << start_offset_s << " s of the bag (--start_offset_s).");

  const double rate = parseRate(argc, argv);
  int64_t t_replay_start_ns = -1;
  std::chrono::steady_clock::time_point wall_start;
  LOG(I, rate > 0.0, "Replaying at " << rate << "x real time (--rate).");

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    if (topic != pc_topic && topic != imu_topic) {
      continue;
    }

    if (start_offset_s > 0.0) {
      if (t_first_ns < 0) t_first_ns = bag_msg->recv_timestamp;
      if (bag_msg->recv_timestamp - t_first_ns < static_cast<int64_t>(start_offset_s * 1e9)) {
        continue;
      }
    }

    // Pace the replay against the wall clock. The reference is taken from the
    // first message actually processed, so this composes with --start_offset_s
    // (no catching up for the skipped part) and a run that falls behind stays
    // behind rather than sprinting to catch up.
    if (rate > 0.0) {
      if (t_replay_start_ns < 0) {
        t_replay_start_ns = bag_msg->recv_timestamp;
        wall_start = std::chrono::steady_clock::now();
      }
      const auto bag_elapsed =
          std::chrono::nanoseconds(bag_msg->recv_timestamp - t_replay_start_ns);
      const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                           bag_elapsed / rate);
      std::this_thread::sleep_until(target);
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    const std::string& type = topic_types[topic];

    if (type == "sensor_msgs/msg/PointCloud2") {
      auto msg = deserialize<sensor_msgs::msg::PointCloud2>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(msg, pointcloud);
      synchronizer->addPointcloud(pointcloud);
      ++n_scans;
    }
#ifdef BIEVR_WITH_LIVOX
    else if (type == "livox_ros_driver2/msg/CustomMsg") {
      auto msg = deserialize<livox_ros_driver2::msg::CustomMsg>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(msg, pointcloud);
      synchronizer->addPointcloud(pointcloud);
      ++n_scans;
    }
#endif
    else if (type == "sensor_msgs/msg/Imu") {
      auto msg = deserialize<sensor_msgs::msg::Imu>(serialized);
      bievr::ImuMeasurement imu;
      bievr::msgToImuMeasurement(msg, imu);
      synchronizer->addImu(imu);
    }

    if (max_scans > 0 && n_scans >= max_scans) {
      LOG(I, "Reached --max_scans (" << max_scans << "), stopping bag replay.");
      break;
    }
  }

  LOG(I, "Done with bag.");
  reader.close();
  LOG(I, "Bag closed");

  pipeline->saveMap();
  pipeline->saveAccumulatedMap();

  rclcpp::shutdown();
  return 0;
}
