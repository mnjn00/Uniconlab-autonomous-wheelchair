#include "static_livox_localization/bounded_cloud_preview.hpp"

#include <cmath>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace static_livox_localization {

BoundedCloudPreview::BoundedCloudPreview(const PreviewConfig& config)
    : config_(config) {
  if (!std::isfinite(config_.voxel_resolution) ||
      config_.voxel_resolution <= 0.0) {
    config_.voxel_resolution = 0.30;
  }
  if (!std::isfinite(config_.max_rate_hz) || config_.max_rate_hz <= 0.0) {
    config_.max_rate_hz = 5.0;
  }
}

bool BoundedCloudPreview::should_publish(double stamp_s) {
  if (!std::isfinite(stamp_s)) return false;
  if (!has_last_publish_stamp_ || stamp_s < last_publish_stamp_s_) {
    has_last_publish_stamp_ = true;
    last_publish_stamp_s_ = stamp_s;
    return true;
  }

  const double minimum_period_s = 1.0 / config_.max_rate_hz;
  if (stamp_s - last_publish_stamp_s_ + 1e-9 < minimum_period_s) {
    return false;
  }
  last_publish_stamp_s_ = stamp_s;
  return true;
}

BoundedCloudPreview::Cloud::Ptr BoundedCloudPreview::downsample(
    const Cloud::ConstPtr& input) const {
  Cloud::Ptr output(new Cloud);
  if (!input || input->empty()) return output;

  Cloud::Ptr finite(new Cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *finite, indices);
  if (finite->empty()) return output;

  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  const float leaf = static_cast<float>(config_.voxel_resolution);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(finite);
  voxel.filter(*output);
  return output;
}

}  // namespace static_livox_localization
