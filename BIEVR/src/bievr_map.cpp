#include "bievr_lio/bievr_map.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

BIEVRMap::BIEVRMap(Config config) : config_(config) {
  // Precompute the offsets to the 8 corners of a voxel for quick access when projecting the voxel
  // into the image
  corner_offsets_ = Eigen::MatrixXd(3, 8);
  int count = 0;
  for (int dx : {0, 1}) {
    for (int dy : {0, 1}) {
      for (int dz : {0, 1}) {
        Eigen::Vector3d corner(dx, dy, dz);
        corner_offsets_.col(count) = corner;
        count++;
      }
    }
  }
  corner_offsets_ *= config_.voxel_size;

  // Store offsets for quick access when searching for neighboring voxels.
  neighbor_offsets_ = getNeighborOffsets(config_.voxel_size);

  // Precompute Gaussian kernel for smoothing
  double radius = 1.0;
  float sigma = computeSigmaFromRadius(radius);
  gauss_kernel_ = buildGaussianKernel(radius, sigma);

  // For quick access
  norm_tol_rad_ = M_PI * config_.norm_tol_deg / 180.0;
  inv_voxel_size_ = 1.0 / config_.voxel_size;
  inv_px_size_ = 1.0 / config_.px_size;
}

bool BIEVRMap::integratePoints(const Pointcloud& cloud, const std::vector<double>* ranges) {
  if (cloud.empty()) {
    LOG(I, "No points in cloud to map.");
    return false;
  }

  LOG(D, "Integrating " << cloud.size() << " points to the map.");

  // Each entry is {voxel hash, point (xyz, plus range/weight in w)}.
  std::vector<std::pair<size_t, Eigen::Vector4d>> hashed_points(cloud.size());

  // Calculate Hash indices for each point
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        hashed_points[i].second.head(3) = cloud[i];
                        if (ranges) {
                          hashed_points[i].second(3) = (*ranges)[i];  // weight by range
                        } else {
                          hashed_points[i].second(3) = 1;
                        }
                        hashed_points[i].first = hashIndex(hashed_points[i].second.head(3));
                      }
                    });

  // Group points by voxel; tie-break on x for a deterministic order within each voxel.
  tbb::parallel_sort(hashed_points.begin(), hashed_points.end(), [](const auto& a, const auto& b) {
    if (a.first != b.first) return a.first < b.first;
    return a.second(0) < b.second(0);
  });

  // Extract unique hash indices
  std::vector<size_t> hash_change_indices;
  hash_change_indices.reserve(hashed_points.size());
  if (!hashed_points.empty()) {
    size_t prev_hash = hashed_points[hashed_points.size() - 1].first;

    for (size_t i = 0; i < hashed_points.size(); ++i) {
      size_t current_hash = hashed_points[i].first;
      if (current_hash != prev_hash) {
        hash_change_indices.push_back(i);
        if (map_.find(current_hash) == map_.end()) {
          voxels_cache_.push_front(current_hash);
          auto res = map_.emplace(current_hash, VoxelEntry{Voxel(), voxels_cache_.begin()});
          res.first->second.voxel.pending_points_.reserve(8);
        }
        prev_hash = current_hash;
      }
    }
  }

  // Update voxels. Resolve each group's voxel iterator once here (in parallel) and reuse it for
  // the serial LRU update below, instead of looking the voxel up in the map again.
  std::vector<decltype(map_)::iterator> voxel_iters(hash_change_indices.size());
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, hash_change_indices.size()),
      [&](const tbb::blocked_range<size_t>& r) {
        std::vector<Eigen::Vector4d> voxel_points;
        for (size_t i = r.begin(); i != r.end(); ++i) {
          int start_idx = hash_change_indices[i];
          int end_idx = (i + 1 < hash_change_indices.size()) ? hash_change_indices[i + 1]
                                                             : hashed_points.size();

          const size_t hash = hashed_points[start_idx].first;
          auto iter = map_.find(hash);
          voxel_iters[i] = iter;
          voxel_points.clear();
          voxel_points.reserve(end_idx - start_idx);
          for (size_t j = start_idx; j < end_idx; ++j) {
            iter->second.voxel.sum_ += hashed_points[j].second.head(3);
            iter->second.voxel.num_points_++;
            iter->second.voxel.outer_sum_ +=
                hashed_points[j].second.head(3) * hashed_points[j].second.head(3).transpose();
            voxel_points.emplace_back(hashed_points[j].second);
          }

          bool valid_before = iter->second.voxel.observed_;
          bool normal_change = updateNormal(iter->second.voxel);

          auto& pending = iter->second.voxel.pending_points_;
          if (!valid_before && iter->second.voxel.observed_) {
            // Just became observed: fold the points accumulated while unobserved into this scan's
            // batch so they get projected once, then release the buffer.
            voxel_points.insert(voxel_points.end(), pending.begin(), pending.end());
            pending.clear();
            pending.shrink_to_fit();
          } else if (!iter->second.voxel.observed_) {
            // Still unobserved: keep accumulating raw points until a normal exists.
            pending.insert(pending.end(), voxel_points.begin(), voxel_points.end());
          }

          updateBumpImage(voxel_points, iter->second.voxel, normal_change);
        }
      });

  // Update LRU Cache
  for (const auto& iter : voxel_iters) {
    voxels_cache_.splice(voxels_cache_.begin(), voxels_cache_, iter->second.lru_it);
  }

  // If max_size exceeded, remove least recently used voxels
  int removal_counter = 0;
  while (map_.size() > config_.max_size && !voxels_cache_.empty()) {
    map_.erase(voxels_cache_.back());
    voxels_cache_.pop_back();
    removal_counter++;
  }

  if (removal_counter > 0) {
    LOG(I, "BIEVRMap exceeded max_size. Removed " << removal_counter << " old voxels.");
  }
  return true;
}

bool BIEVRMap::updateNormal(Voxel& voxel) {
  if (voxel.num_points_ < kNMinValid) return false;

  Point mean = voxel.sum_ / static_cast<double>(voxel.num_points_);
  M3 covariance = (voxel.outer_sum_ - voxel.sum_ * mean.transpose()) /
                  (static_cast<double>(voxel.num_points_) - 1.0);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

  // Smallest eigenvalue -> normal
  Eigen::Vector3d normal = solver.eigenvectors().col(0);  //

  bool update_normal = false;
  if (voxel.observed_) {
    // The previous normal is the Z axis of the old frame
    const Eigen::Vector3d& prev_normal = voxel.T_O_W_.linear().row(2);
    double angle = std::acos(std::abs(prev_normal.dot(normal)));
    if (angle > norm_tol_rad_) {
      update_normal = true;
    }
  } else {
    update_normal = true;
  }
  if (update_normal) {
    // Get the normal vector (Z axis of the new frame)
    Eigen::Vector3d z_axis = normal;
    Eigen::Vector3d x_axis = z_axis.unitOrthogonal();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    // Rotation matrix to align with plane
    Eigen::Matrix3d R_align;
    R_align.col(0) = x_axis;
    R_align.col(1) = y_axis;
    R_align.col(2) = z_axis;
    Transform T_W_O;
    T_W_O.linear() = R_align;
    T_W_O.translation() = mean;
    voxel.T_O_W_ = T_W_O.inverse();
  }

  voxel.observed_ = true;

  return update_normal;
}

bool BIEVRMap::updateBumpImage(const std::vector<Eigen::Vector4d>& points, Voxel& voxel,
                               bool normal_change) {
  if (!voxel.observed_) {
    return false;
  }

  Eigen::MatrixXi changed;
  if (normal_change) {
    // If the normal has changed, we reproject the surface from the old image into the new image
    ImageBounds bounds = computeImageSize(voxel, points[0].head<3>());
    reprojectImage(voxel, bounds, changed);
  } else {
    changed = Eigen::MatrixXi::Zero(voxel.bump_img_.rows(), voxel.bump_img_.cols());
  }

  // Update the pixel values and weights based on the new points
  integratePoints(points, voxel, changed);

  Eigen::MatrixXi changed_dilated =
      Eigen::MatrixXi::Zero(voxel.bump_img_.rows(), voxel.bump_img_.cols());
  if (normal_change) {
    changed_dilated = changed;
  } else {
    // For the smoothing, we also need to consider pixels that are adjacent to the changed pixels
    dilateMask(changed, voxel.bump_weights_, changed_dilated);
  }

  if (config_.smooth) {
    maskedGaussianSmooth(voxel.bump_img_, voxel.bump_weights_, changed_dilated,
                         voxel.bump_smoothed_);
  } else {
    voxel.bump_smoothed_ = voxel.bump_img_;
  }

  computeScore(voxel);

  return true;
}

BIEVRMap::ImageBounds BIEVRMap::computeImageSize(const Voxel& voxel,
                                                 const Eigen::Vector3d& reference_point) const {
  Eigen::Vector3d p_origin = getVoxelOrigin(reference_point);
  Eigen::MatrixXd voxel_corners = corner_offsets_.colwise() + p_origin;
  // Project voxel corners on voxel plane
  Eigen::MatrixXd uv_convers = voxel.T_O_W_.matrix().block<2, 3>(0, 0) * voxel_corners +
                               voxel.T_O_W_.matrix().block<2, 1>(0, 3).replicate(1, 8);
  // Calculate minimum required image size to cover all corners
  ImageBounds bounds;
  bounds.u_min = uv_convers.row(0).minCoeff();
  double u_max = uv_convers.row(0).maxCoeff();
  bounds.v_min = uv_convers.row(1).minCoeff();
  double v_max = uv_convers.row(1).maxCoeff();
  bounds.width = static_cast<int>(std::ceil((u_max - bounds.u_min) * inv_px_size_) + 1);
  bounds.height = static_cast<int>(std::ceil((v_max - bounds.v_min) * inv_px_size_) + 1);
  return bounds;
}

void BIEVRMap::reprojectImage(Voxel& voxel, const ImageBounds& bounds, Eigen::MatrixXi& changed) {
  Eigen::MatrixXf bump_original = voxel.bump_img_;
  Eigen::MatrixXf weights_original = voxel.bump_weights_;
  Transform T_W_C_o = voxel.T_C_W_.inverse();
  voxel.bump_img_.resize(bounds.height, bounds.width);
  voxel.bump_smoothed_.resize(bounds.height, bounds.width);
  voxel.bump_weights_.resize(bounds.height, bounds.width);
  changed.resize(bounds.height, bounds.width);
  voxel.bump_img_.setZero();
  voxel.bump_smoothed_.setZero();
  voxel.bump_weights_.setZero();
  changed.setZero();
  Point p_o_planar(bounds.u_min, bounds.v_min, 0.0);
  Point p_w_o = voxel.T_O_W_.inverse() * p_o_planar;
  Transform T_W_C;
  T_W_C.matrix().topLeftCorner<3, 3>() = voxel.T_O_W_.linear().transpose();
  T_W_C.matrix().topRightCorner<3, 1>() = p_w_o;
  voxel.T_C_W_ = T_W_C.inverse();

  Transform T_C1_C0 = voxel.T_C_W_ * T_W_C_o;
  for (int i = 0; i < bump_original.rows(); ++i) {
    for (int j = 0; j < bump_original.cols(); ++j) {
      if (weights_original(i, j) == 0) continue;
      // Lift the point to 3D using the original bump value, then transform it into the new camera
      // frame and project it
      Point p_O = T_C1_C0 * Point(j * config_.px_size, i * config_.px_size, bump_original(i, j));
      int x = static_cast<int>(std::round(p_O(0) * inv_px_size_));
      int y = static_cast<int>(std::round(p_O(1) * inv_px_size_));

      if (x < 0 || x >= voxel.bump_img_.cols() || y < 0 || y >= voxel.bump_img_.rows()) {
        continue;
      }

      voxel.bump_img_(y, x) = p_O(2);
      voxel.bump_weights_(y, x) = weights_original(i, j);
      changed(y, x) = 1;
    }
  }
}

void BIEVRMap::integratePoints(const std::vector<Eigen::Vector4d>& points, Voxel& voxel,
                               Eigen::MatrixXi& changed) {
  for (const auto& p : points) {
    Point p_O = voxel.T_C_W_.linear() * p.head(3) + voxel.T_C_W_.translation();

    int x = static_cast<int>(std::round(p_O(0) * inv_px_size_));
    int y = static_cast<int>(std::round(p_O(1) * inv_px_size_));

    double mean_old = voxel.bump_img_(y, x);
    double weight = voxel.bump_weights_(y, x);
    // Limit weighting so points close to the sensor don't get too powerful
    double weight_new = config_.weighted ? std::min(0.5, 1. / p(3)) : 1.;
    voxel.bump_weights_(y, x) += weight_new;
    voxel.bump_img_(y, x) = (mean_old * weight + weight_new * p_O(2)) / voxel.bump_weights_(y, x);
    changed(y, x) = 1;
  }
}

void BIEVRMap::dilateMask(const Eigen::MatrixXi& changed, const Eigen::MatrixXf& weights,
                          Eigen::MatrixXi& changed_dilated) {
  for (size_t i = 0; i < changed.rows(); ++i) {
    for (size_t j = 0; j < changed.cols(); ++j) {
      if (changed(i, j) != 1) continue;
      for (int k = -1; k < 2; k++) {
        int i_n = i + k;
        if (i_n < 0 || i_n >= changed.rows()) continue;
        for (int l = -1; l < 2; l++) {
          int j_n = j + l;
          if (j_n < 0 || j_n >= changed.cols()) continue;
          if (weights(i_n, j_n) <= 0.) continue;
          changed_dilated(i_n, j_n) = 1;
        }
      }
    }
  }
}

// Apply Gaussian smoothing only over valid pixels
void BIEVRMap::maskedGaussianSmooth(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                                    const Eigen::MatrixXi& changed, Eigen::MatrixXf& image_smooth) {
  int rows = image.rows();
  int cols = image.cols();
  int radius = gauss_kernel_.rows() / 2;
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      // Skip pixels not marked
      if (!changed(y, x)) {
        continue;
      }
      float weightedSum = 0.0f;
      float weightSum = 0.0f;

      for (int ky = -radius; ky <= radius; ++ky) {
        for (int kx = -radius; kx <= radius; ++kx) {
          int yy = y + ky;
          int xx = x + kx;

          if (xx < 0 || yy < 0 || xx >= cols || yy >= rows) continue;

          if (weights(yy, xx) > 0.) {
            float w = gauss_kernel_(ky + radius, kx + radius);
            weightedSum += image(yy, xx) * w;
            weightSum += w;
          }
        }
      }

      if (weightSum > 0.0f) {
        image_smooth(y, x) = weightedSum / weightSum;
      } else {
        image_smooth(y, x) = 0.0f;
      }
    }
  }
}

void BIEVRMap::computeScore(Voxel& voxel) {
  int total_count = 0;
  double sum_img_dist = 0.0;

  for (int i = 0; i < voxel.bump_img_.rows(); ++i) {
    for (int j = 0; j < voxel.bump_img_.cols(); ++j) {
      if (voxel.bump_weights_(i, j) > 0) {
        total_count++;
        double val = static_cast<double>(voxel.bump_smoothed_(i, j));
        sum_img_dist += std::abs(val);
      }
    }
  }

  voxel.mean_img_dist_ = sum_img_dist / total_count;

  // Discourage voxels with few observed pixels as they have lower probability of giving successful
  // correspondences
  if (total_count < 5) {
    voxel.mean_img_dist_ = 0;
  }
}

const Voxel* BIEVRMap::getVoxel(size_t hash_idx) const {
  auto it = map_.find(hash_idx);
  if (it != map_.end() && it->second.voxel.observed_) {
    return &it->second.voxel;
  }
  return nullptr;
}

Eigen::Vector3i BIEVRMap::getVoxelIdx(const Eigen::Vector3d& point) const {
  Eigen::Vector3i idx;
  idx(0) = floor(point.x() * inv_voxel_size_);
  idx(1) = floor(point.y() * inv_voxel_size_);
  idx(2) = floor(point.z() * inv_voxel_size_);
  return idx;
}

Eigen::Vector3d BIEVRMap::getVoxelOrigin(const Eigen::Vector3d& point) const {
  int hx = static_cast<int>(std::floor(point.x() * inv_voxel_size_));
  int hy = static_cast<int>(std::floor(point.y() * inv_voxel_size_));
  int hz = static_cast<int>(std::floor(point.z() * inv_voxel_size_));
  return Eigen::Vector3d(hx * config_.voxel_size, hy * config_.voxel_size, hz * config_.voxel_size);
}

size_t BIEVRMap::exportMap(const std::string& pcd_path, const std::string& native_path) const {
  // --- Pass 1: count valid pixels across all observed voxels (needed up front so the
  // PCD header's WIDTH/POINTS fields are correct without buffering all points in RAM). ---
  size_t n_points = 0;
  size_t n_voxels = 0;
  for (const auto& kv : map_) {
    const Voxel& voxel = kv.second.voxel;
    if (!voxel.observed_ || voxel.bump_weights_.size() == 0) continue;
    n_voxels++;
    for (int i = 0; i < voxel.bump_weights_.rows(); ++i) {
      for (int j = 0; j < voxel.bump_weights_.cols(); ++j) {
        if (voxel.bump_weights_(i, j) > 0) n_points++;
      }
    }
  }

  // --- Pass 2: stream the binary PCD. ---
  {
    std::ofstream pcd(pcd_path, std::ios::binary | std::ios::trunc);
    if (!pcd.is_open()) {
      LOG(E, "exportMap: could not open '" << pcd_path << "' for writing.");
      return 0;
    }
    pcd << "# .PCD v0.7 - BIEVR-LIO map export\n";
    pcd << "VERSION 0.7\n";
    pcd << "FIELDS x y z intensity\n";
    pcd << "SIZE 4 4 4 4\n";
    pcd << "TYPE F F F F\n";
    pcd << "COUNT 1 1 1 1\n";
    pcd << "WIDTH " << n_points << "\n";
    pcd << "HEIGHT 1\n";
    pcd << "VIEWPOINT 0 0 0 1 0 0 0\n";
    pcd << "POINTS " << n_points << "\n";
    pcd << "DATA binary\n";

    for (const auto& kv : map_) {
      const Voxel& voxel = kv.second.voxel;
      if (!voxel.observed_ || voxel.bump_weights_.size() == 0) continue;
      const Transform T_W_C = voxel.T_C_W_.inverse();
      for (int i = 0; i < voxel.bump_img_.rows(); ++i) {
        for (int j = 0; j < voxel.bump_img_.cols(); ++j) {
          const float w = voxel.bump_weights_(i, j);
          if (w <= 0) continue;
          const Point p_C(j * config_.px_size, i * config_.px_size, voxel.bump_img_(i, j));
          const Point p_W = T_W_C * p_C;
          const float xyz_w[4] = {static_cast<float>(p_W(0)), static_cast<float>(p_W(1)),
                                  static_cast<float>(p_W(2)), w};
          pcd.write(reinterpret_cast<const char*>(xyz_w), sizeof(xyz_w));
        }
      }
    }
  }

  // --- Native bump-map dump: raw voxels (pose + bump-image / weight matrices). ---
  {
    std::ofstream native(native_path, std::ios::binary | std::ios::trunc);
    if (!native.is_open()) {
      LOG(E, "exportMap: could not open '" << native_path << "' for writing.");
      return n_points;
    }
    const char magic[8] = {'B', 'I', 'E', 'V', 'R', 'M', 'P', '\0'};
    native.write(magic, sizeof(magic));
    const uint32_t version = kNativeFormatVersion;
    native.write(reinterpret_cast<const char*>(&version), sizeof(version));
    const double voxel_size = config_.voxel_size;
    const double px_size = config_.px_size;
    native.write(reinterpret_cast<const char*>(&voxel_size), sizeof(voxel_size));
    native.write(reinterpret_cast<const char*>(&px_size), sizeof(px_size));
    const uint64_t n_voxels_u64 = static_cast<uint64_t>(n_voxels);
    native.write(reinterpret_cast<const char*>(&n_voxels_u64), sizeof(n_voxels_u64));

    for (const auto& kv : map_) {
      const Voxel& voxel = kv.second.voxel;
      if (!voxel.observed_ || voxel.bump_weights_.size() == 0) continue;

      double T_C_W[12];
      double T_O_W[12];
      const auto T_C_W_mat = voxel.T_C_W_.matrix();
      const auto T_O_W_mat = voxel.T_O_W_.matrix();
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
          T_C_W[r * 4 + c] = T_C_W_mat(r, c);
          T_O_W[r * 4 + c] = T_O_W_mat(r, c);
        }
      }
      native.write(reinterpret_cast<const char*>(T_C_W), sizeof(T_C_W));
      native.write(reinterpret_cast<const char*>(T_O_W), sizeof(T_O_W));

      const V3 centroid = voxel.sum_ / static_cast<double>(voxel.num_points_);
      double centroid_arr[3] = {centroid(0), centroid(1), centroid(2)};
      native.write(reinterpret_cast<const char*>(centroid_arr), sizeof(centroid_arr));

      const Eigen::Vector3d normal = voxel.T_O_W_.linear().row(2);
      double normal_arr[3] = {normal(0), normal(1), normal(2)};
      native.write(reinterpret_cast<const char*>(normal_arr), sizeof(normal_arr));

      const uint64_t num_points = static_cast<uint64_t>(voxel.num_points_);
      native.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

      const int32_t rows = static_cast<int32_t>(voxel.bump_img_.rows());
      const int32_t cols = static_cast<int32_t>(voxel.bump_img_.cols());
      native.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
      native.write(reinterpret_cast<const char*>(&cols), sizeof(cols));

      std::vector<float> row_major_img(static_cast<size_t>(rows) * cols);
      std::vector<float> row_major_weights(static_cast<size_t>(rows) * cols);
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          row_major_img[static_cast<size_t>(i) * cols + j] = voxel.bump_img_(i, j);
          row_major_weights[static_cast<size_t>(i) * cols + j] = voxel.bump_weights_(i, j);
        }
      }
      native.write(reinterpret_cast<const char*>(row_major_img.data()),
                   static_cast<std::streamsize>(row_major_img.size() * sizeof(float)));
      native.write(reinterpret_cast<const char*>(row_major_weights.data()),
                   static_cast<std::streamsize>(row_major_weights.size() * sizeof(float)));

      // --- Voxel index + smoothed image. ---
      // The index makes the hash key explicit instead of re-deriving it from the
      // centroid on load, and the smoothed image is what the registration
      // actually samples, so writing it keeps import lossless.
      const Eigen::Vector3i idx = getVoxelIdx(centroid);
      const int32_t idx_arr[3] = {idx.x(), idx.y(), idx.z()};
      native.write(reinterpret_cast<const char*>(idx_arr), sizeof(idx_arr));

      std::vector<float> row_major_smoothed(static_cast<size_t>(rows) * cols);
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          row_major_smoothed[static_cast<size_t>(i) * cols + j] = voxel.bump_smoothed_(i, j);
        }
      }
      native.write(reinterpret_cast<const char*>(row_major_smoothed.data()),
                   static_cast<std::streamsize>(row_major_smoothed.size() * sizeof(float)));
    }
  }

  LOG(I, "exportMap: wrote " << n_voxels << " voxels / " << n_points << " points -> " << pcd_path
                             << " (+ " << native_path << ")");
  return n_points;
}

void BIEVRMap::reconstructPoints(Pointcloud& out, size_t stride) const {
  if (stride == 0) stride = 1;
  // Same reconstruction as exportMap's PCD pass (kept separate so that one can
  // keep streaming straight to disk instead of buffering the whole cloud).
  size_t n_valid = 0;
  for (const auto& kv : map_) {
    const Voxel& voxel = kv.second.voxel;
    if (!voxel.observed_ || voxel.bump_weights_.size() == 0) continue;
    for (int i = 0; i < voxel.bump_weights_.rows(); ++i) {
      for (int j = 0; j < voxel.bump_weights_.cols(); ++j) {
        if (voxel.bump_weights_(i, j) > 0) n_valid++;
      }
    }
  }

  out.clear();
  out.resize((n_valid + stride - 1) / stride);
  size_t seen = 0;
  size_t written = 0;
  for (const auto& kv : map_) {
    const Voxel& voxel = kv.second.voxel;
    if (!voxel.observed_ || voxel.bump_weights_.size() == 0) continue;
    const Transform T_W_C = voxel.T_C_W_.inverse();
    for (int i = 0; i < voxel.bump_img_.rows(); ++i) {
      for (int j = 0; j < voxel.bump_img_.cols(); ++j) {
        if (voxel.bump_weights_(i, j) <= 0) continue;
        if (seen++ % stride != 0) continue;
        if (written >= out.size()) break;
        const Point p_C(j * config_.px_size, i * config_.px_size, voxel.bump_img_(i, j));
        out[written++] = T_W_C * p_C;
      }
    }
  }
  out.resize(written);
}

size_t BIEVRMap::importMap(const std::string& native_path) {
  std::ifstream native(native_path, std::ios::binary);
  if (!native.is_open()) {
    LOG(E, "importMap: could not open '" << native_path << "'.");
    return 0;
  }

  char magic[8] = {0};
  native.read(magic, sizeof(magic));
  const char expected[8] = {'B', 'I', 'E', 'V', 'R', 'M', 'P', '\0'};
  if (!native || std::memcmp(magic, expected, sizeof(magic)) != 0) {
    LOG(E, "importMap: '" << native_path << "' is not a BIEVR bump map.");
    return 0;
  }

  uint32_t version = 0;
  double file_voxel_size = 0.0;
  double file_px_size = 0.0;
  uint64_t n_voxels = 0;
  native.read(reinterpret_cast<char*>(&version), sizeof(version));
  native.read(reinterpret_cast<char*>(&file_voxel_size), sizeof(file_voxel_size));
  native.read(reinterpret_cast<char*>(&file_px_size), sizeof(file_px_size));
  native.read(reinterpret_cast<char*>(&n_voxels), sizeof(n_voxels));
  if (!native) {
    LOG(E, "importMap: truncated header in '" << native_path << "'.");
    return 0;
  }
  if (version != kNativeFormatVersion) {
    LOG(E, "importMap: unsupported format version " << version << " (this build reads and writes v"
                                                    << kNativeFormatVersion << " only).");
    return 0;
  }

  // A config that disagrees with the file would silently mis-address
  // and mis-scale the whole map. Refuse rather than reinterpret.
  constexpr double kSizeTol = 1e-9;
  if (std::abs(file_voxel_size - config_.voxel_size) > kSizeTol ||
      std::abs(file_px_size - config_.px_size) > kSizeTol) {
    LOG(E, "importMap: map geometry (voxel "
               << file_voxel_size << " m, pixel " << file_px_size << " m) does not match the config ("
               << config_.voxel_size << " m / " << config_.px_size << " m).");
    return 0;
  }

  map_.clear();
  voxels_cache_.clear();

  size_t n_loaded = 0;
  size_t n_duplicate = 0;
  for (uint64_t v = 0; v < n_voxels; ++v) {
    double T_C_W[12];
    double T_O_W[12];
    double centroid_arr[3];
    double normal_arr[3];
    uint64_t num_points = 0;
    int32_t rows = 0;
    int32_t cols = 0;
    native.read(reinterpret_cast<char*>(T_C_W), sizeof(T_C_W));
    native.read(reinterpret_cast<char*>(T_O_W), sizeof(T_O_W));
    native.read(reinterpret_cast<char*>(centroid_arr), sizeof(centroid_arr));
    native.read(reinterpret_cast<char*>(normal_arr), sizeof(normal_arr));  // = T_O_W row 2
    native.read(reinterpret_cast<char*>(&num_points), sizeof(num_points));
    native.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    native.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    if (!native || rows <= 0 || cols <= 0 || num_points == 0) {
      LOG(E, "importMap: corrupt voxel record " << v << " in '" << native_path << "'.");
      return 0;
    }

    const size_t n_px = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    std::vector<float> img(n_px);
    std::vector<float> weights(n_px);
    native.read(reinterpret_cast<char*>(img.data()),
                static_cast<std::streamsize>(n_px * sizeof(float)));
    native.read(reinterpret_cast<char*>(weights.data()),
                static_cast<std::streamsize>(n_px * sizeof(float)));

    int32_t idx_arr[3];
    native.read(reinterpret_cast<char*>(idx_arr), sizeof(idx_arr));
    const Eigen::Vector3i voxel_idx(idx_arr[0], idx_arr[1], idx_arr[2]);
    std::vector<float> smoothed(n_px);
    native.read(reinterpret_cast<char*>(smoothed.data()),
                static_cast<std::streamsize>(n_px * sizeof(float)));
    if (!native) {
      LOG(E, "importMap: truncated voxel record " << v << " in '" << native_path << "'.");
      return 0;
    }

    const V3 centroid(centroid_arr[0], centroid_arr[1], centroid_arr[2]);

    Voxel voxel;
    voxel.observed_ = true;
    Eigen::Matrix4d T_C_W_mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_O_W_mat = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        T_C_W_mat(r, c) = T_C_W[r * 4 + c];
        T_O_W_mat(r, c) = T_O_W[r * 4 + c];
      }
    }
    voxel.T_C_W_.matrix() = T_C_W_mat;
    voxel.T_O_W_.matrix() = T_O_W_mat;
    voxel.num_points_ = static_cast<size_t>(num_points);
    voxel.sum_ = centroid * static_cast<double>(num_points);
    // outer_sum_ is not serialized (updateNormal is the only consumer); left at
    // zero, which is why an imported map must not be used for mapping.
    voxel.bump_img_.resize(rows, cols);
    voxel.bump_weights_.resize(rows, cols);
    voxel.bump_smoothed_.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const size_t k = static_cast<size_t>(i) * cols + j;
        voxel.bump_img_(i, j) = img[k];
        voxel.bump_weights_(i, j) = weights[k];
        voxel.bump_smoothed_(i, j) = smoothed[k];
      }
    }

    computeScore(voxel);  // mean_img_dist_, used by informed sampling

    const size_t hash = hashIndexVoxel(voxel_idx);
    if (map_.find(hash) != map_.end()) {
      n_duplicate++;
      continue;
    }
    voxels_cache_.push_front(hash);
    map_.emplace(hash, VoxelEntry{std::move(voxel), voxels_cache_.begin()});
    n_loaded++;
  }

  if (n_duplicate > 0) {
    LOG(W, "importMap: skipped " << n_duplicate << " voxels with duplicate hash keys.");
  }
  LOG(I, "importMap: loaded " << n_loaded << " voxels (format v" << version << ", voxel "
                              << file_voxel_size << " m, pixel " << file_px_size << " m) from "
                              << native_path);
  // Eviction only ever runs inside integratePoints, so an over-sized map is kept
  // whole while frozen; it would start shedding voxels the moment mapping resumed.
  LOG(W, n_loaded > config_.max_size,
      "importMap: loaded " << n_loaded << " voxels, more than map.max_size (" << config_.max_size
                           << "). Raise max_size before mapping into this map.");
  return n_loaded;
}

bool BIEVRMap::nearestVoxel(const Eigen::Vector3d& point, size_t& result) const {
  double min_dist = std::numeric_limits<double>::max();
  bool found = false;
  // Select nearest voxel based on distance to voxel centroid
  for (const auto& offset : neighbor_offsets_) {
    Eigen::Vector3d nearby_point = point + offset;
    size_t hash_idx = hashIndex(nearby_point);
    auto voxel = getVoxel(hash_idx);
    if (!voxel) continue;
    Eigen::Vector3d centroid = voxel->sum_ / static_cast<double>(voxel->num_points_);
    double sqdistance = (point - centroid).squaredNorm();
    if (sqdistance > min_dist) continue;
    min_dist = sqdistance;
    result = hash_idx;
    found = true;
  }
  return found;
}

}  // namespace bievr
