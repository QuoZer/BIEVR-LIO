// Build a BIEVR bump map directly from an accumulated point cloud, instead of
// running a live mapping pass over a bag.
//
// BIEVRMap::integratePoints needs no sensor pose, no timestamps and no IMU: it
// hashes points into voxels, fits a plane per voxel from running sums, and
// projects points into that voxel's bump image. Everything the .bumpmap format
// needs is derivable from world points alone, so this tool reads one or more
// PCD/PLY clouds, feeds them through the same BIEVRMap the live pipeline uses,
// and calls the same BIEVRMap::exportMap. See
// docs/plans/SPEC-bumpmap-from-pcd.md for the full rationale.
//
// This is deliberately NOT a ROS executable: it lives in the plain-CMake
// bievr_lio package so a converted map can be produced without a workspace
// build, a bag, or a live mapping run.

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/config_loader.h"
#include "bievr_lio/log++.h"

namespace {

namespace fs = std::filesystem;

constexpr size_t kDefaultChunkSize = 2000000;
constexpr size_t kDefaultStride = 1;
// Mirrors BIEVRMap::kNMinValid (bievr_map.h, private): the minimum number of
// points a voxel needs before updateNormal() will fit a plane and mark it
// observed. Kept in sync by hand since the constant isn't exposed publicly;
// used here only to fail a degenerate whole-input early with a clear message
// instead of silently exporting an empty map.
constexpr size_t kNMinValid = 4;

void printUsage(std::ostream& os) {
  os <<
      R"(Usage: bumpmap_from_pcd --input <cloud> [--input <cloud> ...] --output-dir <dir>
                         --config <params.yaml> [--sensor-config <sensor.yaml>]
                         [--override key=value ...] [--chunk-size N] [--stride N]

Build a BIEVR bump map (<output-dir>/map.bumpmap + map.pcd) directly from an
accumulated point cloud (PCD ascii/binary/binary_compressed, or PLY), instead
of running a live mapping pass over a bag. Writes the same two paths, in the
same order, that BIEVRMap::exportMap always writes, so the output directory is
layout-identical to a mapping run's and every existing consumer
(run_bievr_loc.sh, scripts/load_bumpmap.py, scripts/view_map.py,
scripts/compare_bumpmap.py) works on it unmodified.

Options:
  --input PATH          Input cloud (PCD or PLY). Repeatable; multiple inputs
                         are concatenated in the order given. Only xyz is used
                         -- an intensity field or any other extra field is
                         dropped. Non-finite points are discarded (counted and
                         logged), never passed into the map.
  --output-dir DIR       Output directory (created if missing). Written files:
                         DIR/map.bumpmap, DIR/map.pcd, DIR/.run_config/sensor.yaml
                         (only when --sensor-config or --override is used).
  --config PATH          params.yaml (required). map.voxel_size_m /
                         map.pixel_size_m here (or overridden by
                         --sensor-config / --override, which win per leaf) fix
                         the output map's geometry. Whatever config later
                         imports this file must resolve to the same two values:
                         BIEVRMap::importMap refuses a mismatch bigger than 1e-9.
  --sensor-config PATH   Optional sensor config, merged over --config the same
                         way run_bievr_nora.sh's OVERRIDES does (sensor file
                         wins per leaf key).
  --override K=V         Dotted-path config override (e.g.
                         map.voxel_size_m=0.25), repeatable, merged the same
                         way run_bievr_nora.sh's OVERRIDES does. The value is
                         parsed as YAML, so bare numbers/bools are typed and
                         anything else is a string.
  --chunk-size N         Points per BIEVRMap::integratePoints() call (default
                         2000000). NOT free: per-voxel normals come from
                         order-independent running sums and do not depend on
                         chunk size, but the bump image is built incrementally
                         and reprojected whenever a voxel's normal moves more
                         than map.normal_tolerance_deg, so a different
                         --chunk-size gives a slightly different bump image.
                         State the chunk size alongside any result derived from
                         this tool's output.
  --stride N             Keep every N-th point after concatenation (default 1),
                         for quick low-resolution trials.
  --help, -h              Print this message and exit.

Weighting: integratePoints() is always called with ranges = nullptr, because an
accumulated world-frame cloud carries no sensor origin or per-point timestamp
to recover a range from. Every point therefore gets weight w=1 and, under the
default map.weighted: true, contributes a uniform 0.5 to each pixel it lands
on. Bump *heights* are unaffected (a constant weight cancels out of the running
weighted mean), but the absolute scale of bump_weights_ -- exported as the
output PCD's intensity field -- is not on the same scale as a map built online
from ranged sensor data. This tool never flips map.weighted to false and never
invents a sensor origin.

The output is localization-only: BIEVRMap::outer_sum_ is not serialized by
exportMap, so a map produced here -- like any imported map -- must not be used
to resume mapping.
)";
}

struct Args {
  std::vector<std::string> inputs;
  std::string output_dir;
  std::string config;
  std::string sensor_config;
  std::vector<std::pair<std::string, std::string>> overrides;
  size_t chunk_size = kDefaultChunkSize;
  size_t stride = kDefaultStride;
};

// Parses argv into Args. Returns false (after printing a message to stderr) on
// a malformed command line; the caller should exit non-zero without going any
// further. `want_help` is set and the function returns true without validating
// required flags when --help/-h was seen, so `bumpmap_from_pcd --help` always
// succeeds even with no other arguments.
bool parseArgs(int argc, char** argv, Args& args, bool& want_help) {
  want_help = false;
  auto value = [&](int& i, const char* flag) -> std::string {
    if (i + 1 >= argc) {
      std::cerr << "Missing value after " << flag << ".\n";
      return "";
    }
    return argv[++i];
  };
  bool ok = true;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      want_help = true;
    } else if (arg == "--input") {
      std::string v = value(i, "--input");
      if (v.empty() && ok) ok = false;
      args.inputs.push_back(v);
    } else if (arg == "--output-dir") {
      args.output_dir = value(i, "--output-dir");
      if (args.output_dir.empty()) ok = false;
    } else if (arg == "--config") {
      args.config = value(i, "--config");
      if (args.config.empty()) ok = false;
    } else if (arg == "--sensor-config") {
      args.sensor_config = value(i, "--sensor-config");
      if (args.sensor_config.empty()) ok = false;
    } else if (arg == "--override") {
      std::string kv = value(i, "--override");
      const size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        std::cerr << "--override must be key=value, got '" << kv << "'.\n";
        ok = false;
        continue;
      }
      args.overrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
    } else if (arg == "--chunk-size") {
      const std::string v = value(i, "--chunk-size");
      try {
        long long n = std::stoll(v);
        if (n <= 0) throw std::invalid_argument("non-positive");
        args.chunk_size = static_cast<size_t>(n);
      } catch (const std::exception&) {
        std::cerr << "--chunk-size must be a positive integer, got '" << v << "'.\n";
        ok = false;
      }
    } else if (arg == "--stride") {
      const std::string v = value(i, "--stride");
      try {
        long long n = std::stoll(v);
        if (n <= 0) throw std::invalid_argument("non-positive");
        args.stride = static_cast<size_t>(n);
      } catch (const std::exception&) {
        std::cerr << "--stride must be a positive integer, got '" << v << "'.\n";
        ok = false;
      }
    } else {
      std::cerr << "Unrecognized argument '" << arg << "'.\n";
      ok = false;
    }
  }
  return ok;
}

std::string lowerExt(const std::string& path) {
  fs::path p(path);
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  return ext;
}

// Assigns `parts[idx..]` under `node`, recursing through freshly-indexed
// child handles. Deliberately NOT "YAML::Node child = node; child =
// child[key];" in a loop: a YAML::Node obtained via plain copy-construction
// from `node` aliases node's own underlying storage, so reassigning that same
// variable to one of its own children overwrites (and permanently loses) the
// rest of node's content instead of descending into the child. Taking the
// child directly as `node[parts[idx]]` avoids ever aliasing a node into
// itself.
void setDottedLeaf(YAML::Node node, const std::vector<std::string>& parts, size_t idx,
                   const std::string& raw_value) {
  if (idx + 1 == parts.size()) {
    node[parts[idx]] = YAML::Load(raw_value);
    return;
  }
  setDottedLeaf(node[parts[idx]], parts, idx + 1, raw_value);
}

// Walks a dotted key path (e.g. "map.voxel_size_m") into `root`, creating
// intermediate maps as needed, and assigns the leaf. The value is parsed as
// YAML so bare numbers/bools come out typed, matching run_bievr_nora.sh's
// `yaml.safe_load(raw)` treatment of the same OVERRIDES syntax.
void applyDottedOverride(YAML::Node& root, const std::string& dotted_key, const std::string& raw_value) {
  std::vector<std::string> parts;
  size_t start = 0;
  while (true) {
    const size_t dot = dotted_key.find('.', start);
    if (dot == std::string::npos) {
      parts.push_back(dotted_key.substr(start));
      break;
    }
    parts.push_back(dotted_key.substr(start, dot - start));
    start = dot + 1;
  }
  setDottedLeaf(root, parts, 0, raw_value);
}

// The fixed-size header BIEVRMap::exportMap writes at the start of a native
// dump: magic, version, voxel_size, px_size, n_voxels (36 bytes). Re-reading it
// after export is a read-only sanity check -- it does not touch BIEVRMap.
struct BumpmapHeader {
  uint32_t version = 0;
  double voxel_size = 0.0;
  double px_size = 0.0;
  uint64_t n_voxels = 0;
};

bool readBumpmapHeader(const std::string& path, BumpmapHeader& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  char magic[8];
  f.read(magic, sizeof(magic));
  f.read(reinterpret_cast<char*>(&out.version), sizeof(out.version));
  f.read(reinterpret_cast<char*>(&out.voxel_size), sizeof(out.voxel_size));
  f.read(reinterpret_cast<char*>(&out.px_size), sizeof(out.px_size));
  f.read(reinterpret_cast<char*>(&out.n_voxels), sizeof(out.n_voxels));
  return static_cast<bool>(f);
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  bool want_help = false;
  if (!parseArgs(argc, argv, args, want_help)) {
    printUsage(std::cerr);
    return 1;
  }
  if (want_help) {
    printUsage(std::cout);
    return 0;
  }
  if (args.inputs.empty()) {
    std::cerr << "At least one --input is required.\n";
    printUsage(std::cerr);
    return 1;
  }
  if (args.output_dir.empty()) {
    std::cerr << "--output-dir is required.\n";
    printUsage(std::cerr);
    return 1;
  }
  if (args.config.empty()) {
    std::cerr << "--config is required.\n";
    printUsage(std::cerr);
    return 1;
  }
  if (!fs::exists(args.config)) {
    LOG(E, "--config does not exist: " << args.config);
    return 1;
  }
  if (!args.sensor_config.empty() && !fs::exists(args.sensor_config)) {
    LOG(E, "--sensor-config does not exist: " << args.sensor_config);
    return 1;
  }
  for (const auto& input : args.inputs) {
    if (!fs::exists(input)) {
      LOG(E, "--input does not exist: " << input);
      return 1;
    }
  }

  std::error_code mkdir_ec;
  fs::create_directories(args.output_dir, mkdir_ec);
  if (mkdir_ec) {
    LOG(E, "Could not create --output-dir '" << args.output_dir << "': " << mkdir_ec.message());
    return 1;
  }

  // --- Assemble the config the same way process_bag.cpp does (config_loader.h
  // R1): loadConfigFromArgs merges {params, sensor} with the sensor file
  // winning per leaf. --override is folded into a copy of the sensor config
  // first -- the same "merge as YAML into a copy" technique
  // run_bievr_nora.sh's OVERRIDES uses -- so it goes through that same merge
  // path rather than being a separate mechanism. ---
  std::string sensor_config_for_loader = args.sensor_config;
  if (!args.overrides.empty()) {
    YAML::Node merged;
    if (!args.sensor_config.empty()) {
      try {
        merged = YAML::LoadFile(args.sensor_config);
      } catch (const std::exception& e) {
        LOG(E, "Failed to parse --sensor-config '" << args.sensor_config << "': " << e.what());
        return 1;
      }
    }
    for (const auto& [key, value] : args.overrides) {
      applyDottedOverride(merged, key, value);
    }
    const fs::path run_config_dir = fs::path(args.output_dir) / ".run_config";
    std::error_code ec;
    fs::create_directories(run_config_dir, ec);
    if (ec) {
      LOG(E, "Could not create '" << run_config_dir.string() << "': " << ec.message());
      return 1;
    }
    const fs::path merged_path = run_config_dir / "sensor.yaml";
    std::ofstream out(merged_path);
    if (!out) {
      LOG(E, "Could not write merged sensor config to '" << merged_path.string() << "'.");
      return 1;
    }
    out << "# Generated by bumpmap_from_pcd -- edits here are overwritten.\n" << merged;
    out.close();
    sensor_config_for_loader = merged_path.string();
    LOG(I, "Applied " << args.overrides.size() << " --override value(s), merged into '"
                       << merged_path.string() << "'.");
  }

  std::vector<std::string> loader_args = {"bumpmap_from_pcd", "--params_file", args.config};
  if (!sensor_config_for_loader.empty()) {
    loader_args.push_back("--sensor_config_file");
    loader_args.push_back(sensor_config_for_loader);
  }
  bievr::Config config;
  if (!bievr::loadConfigFromArgs(loader_args, config)) {
    LOG(E, "Failed to load config from '" << args.config
                                          << (sensor_config_for_loader.empty()
                                                  ? "'"
                                                  : "' + '" + sensor_config_for_loader + "'"));
    return 1;
  }
  const bievr::BIEVRMap::Config& map_cfg = config.pipeline_config.map;

  // --- R3: read and concatenate the inputs, dropping non-finite points. ---
  std::vector<Eigen::Vector3d> points;
  size_t n_nonfinite = 0;
  for (const auto& input : args.inputs) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const std::string ext = lowerExt(input);
    int ret = 0;
    if (ext == ".ply") {
      ret = pcl::io::loadPLYFile(input, cloud);
    } else {
      // Handles ASCII, binary and binary_compressed PCD transparently.
      ret = pcl::io::loadPCDFile(input, cloud);
    }
    if (ret != 0) {
      LOG(E, "Failed to read point cloud '" << input << "' (unrecognized/corrupt "
                                            << (ext.empty() ? "(no extension)" : ext) << " file).");
      return 1;
    }
    LOG(I, "Read " << cloud.size() << " points from '" << input << "'.");
    points.reserve(points.size() + cloud.size());
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++n_nonfinite;
        continue;
      }
      points.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
    }
  }
  LOG(W, n_nonfinite > 0, "Discarded " << n_nonfinite << " non-finite point(s) across all inputs.");

  if (args.stride > 1) {
    std::vector<Eigen::Vector3d> strided;
    strided.reserve((points.size() + args.stride - 1) / args.stride);
    for (size_t i = 0; i < points.size(); i += args.stride) strided.push_back(points[i]);
    LOG(I, "Stride " << args.stride << ": " << points.size() << " -> " << strided.size() << " points.");
    points.swap(strided);
  }

  if (points.empty()) {
    LOG(E, "No valid points to integrate: input cloud(s) are empty (or every point was "
           "non-finite / dropped by --stride).");
    return 1;
  }
  if (points.size() < kNMinValid) {
    LOG(E, "Only " << points.size() << " point(s) survive filtering, fewer than the minimum ("
                   << kNMinValid << ") a voxel needs to fit a plane. Refusing to write an "
                                    "all-empty map.");
    return 1;
  }

  // --- R2: bounded-memory chunked integration. ---
  const size_t total = points.size();
  const size_t n_chunks = (total + args.chunk_size - 1) / args.chunk_size;
  LOG(I, "Integrating " << total << " points in " << n_chunks << " chunk(s) of up to "
                        << args.chunk_size << " points each (--chunk-size). Chunking changes the "
                                              "bump image at reprojection boundaries -- see --help.");

  bievr::BIEVRMap map(map_cfg);
  for (size_t start = 0; start < total; start += args.chunk_size) {
    const size_t end = std::min(start + args.chunk_size, total);
    bievr::Pointcloud chunk;
    chunk.resize(end - start);
    for (size_t i = start; i < end; ++i) {
      chunk[static_cast<Eigen::Index>(i - start)] = points[i];
    }
    // ranges = nullptr (R4): no sensor origin survives an accumulated
    // world-frame cloud, so every point gets uniform weight (see --help).
    map.integratePoints(chunk, nullptr);
  }

  // --- R1: log the geometry actually used, and R4/R5 caveats, once on exit. ---
  LOG(I, "Map geometry used: voxel_size_m=" << map_cfg.voxel_size << " pixel_size_m=" << map_cfg.px_size
                                            << " normal_tolerance_deg=" << map_cfg.norm_tol_deg
                                            << " max_size=" << map_cfg.max_size);
  LOG(I, "map.weighted=" << (map_cfg.weighted ? "true" : "false")
                        << ": ranges=nullptr was passed to every integratePoints() call, so all "
                           "points got uniform weight (w=1, i.e. a flat 0.5 contribution under "
                           "weighted=true). Bump heights are unaffected; bump_weights_ (the "
                           "exported PCD's intensity field) is not on the same scale as an online, "
                           "ranged map.");
  LOG(I, "This map is localization-only: BIEVRMap::outer_sum_ is not serialized by exportMap, so "
         "it -- like any imported map -- must not be used to resume mapping.");

  const std::string pcd_path = (fs::path(args.output_dir) / "map.pcd").string();
  const std::string native_path = (fs::path(args.output_dir) / "map.bumpmap").string();
  const size_t n_points = map.exportMap(pcd_path, native_path);

  if (!fs::exists(native_path) || fs::file_size(native_path) == 0) {
    LOG(E, "exportMap did not produce a usable native map at '" << native_path
                                                                 << "'; see errors above.");
    return 1;
  }
  if (!fs::exists(pcd_path) || fs::file_size(pcd_path) == 0) {
    LOG(E, "exportMap did not produce a usable PCD at '" << pcd_path << "'; see errors above.");
    return 1;
  }

  BumpmapHeader written{};
  if (readBumpmapHeader(native_path, written)) {
    LOG(I, "Written file geometry: voxel_size=" << written.voxel_size << " px_size=" << written.px_size
                                                << " n_voxels=" << written.n_voxels << " (format v"
                                                << written.version << ") -> " << native_path);
  } else {
    LOG(W, "Could not re-read the header of '" << native_path << "' to confirm what was written.");
  }

  LOG(I, "Done: " << n_points << " points, " << written.n_voxels << " voxels -> " << pcd_path << " + "
                  << native_path);
  return 0;
}
