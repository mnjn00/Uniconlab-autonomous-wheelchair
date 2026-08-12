#include "static_livox_localization/rolling_submap.hpp"

#include <cmath>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace static_livox_localization {

std::size_t filter_dynamic_returns(
    pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std::vector<DynamicBox>& boxes,
    double margin_m,
    double max_dropped_fraction,
    bool* refused) {
  if (refused) *refused = false;
  if (boxes.empty() || cloud.empty()) return 0;
  std::vector<char> drop(cloud.size(), 0);
  std::size_t dropped = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = cloud.points[i];
    for (const auto& box : boxes) {
      if (std::abs(p.x - box.centre.x()) <= box.half_extent.x() + margin_m &&
          std::abs(p.y - box.centre.y()) <= box.half_extent.y() + margin_m &&
          std::abs(p.z - box.centre.z()) <= box.half_extent.z() + margin_m) {
        drop[i] = 1;
        ++dropped;
        break;
      }
    }
  }
  if (dropped == 0) return 0;
  const double fraction =
      static_cast<double>(dropped) / static_cast<double>(cloud.size());
  if (fraction > max_dropped_fraction) {
    if (refused) *refused = true;
    return 0;
  }
  pcl::PointCloud<pcl::PointXYZI> kept;
  kept.reserve(cloud.size() - dropped);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (!drop[i]) kept.push_back(cloud.points[i]);
  }
  kept.header = cloud.header;
  kept.is_dense = cloud.is_dense;
  cloud.swap(kept);
  return dropped;
}

RollingSubmap::RollingSubmap(const RollingSubmapConfig& config)
    : config_(config) {}

bool RollingSubmap::add_sample(const Cloud::ConstPtr& cloud,
                               double cloud_stamp_s,
                               const Eigen::Isometry3d& odom_T_base,
                               double odom_stamp_s,
                               const std::string& cloud_frame) {
  if (!cloud || cloud->empty() || !std::isfinite(cloud_stamp_s) ||
      !std::isfinite(odom_stamp_s)) {
    return false;
  }
  if (std::abs(cloud_stamp_s - odom_stamp_s) > config_.max_stamp_skew_s) {
    return false;
  }
  if (!config_.expected_cloud_frame.empty() &&
      cloud_frame != config_.expected_cloud_frame) {
    return false;
  }
  if (!samples_.empty() && cloud_stamp_s < samples_.back().stamp_s) {
    return false;
  }

  Cloud::Ptr finite(new Cloud);
  std::vector<int> kept;
  pcl::removeNaNFromPointCloud(*cloud, *finite, kept);
  if (finite->empty()) return false;

  if (config_.max_stored_points > 0 &&
      finite->size() > config_.max_stored_points) {
    finite->resize(config_.max_stored_points);
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1;
  }

  Sample sample;
  sample.cloud = finite;
  sample.stamp_s = cloud_stamp_s;
  sample.odom_T_base = odom_T_base;
  stored_point_count_ += sample.cloud->size();
  samples_.push_back(std::move(sample));
  trim(cloud_stamp_s);
  return true;
}

RollingSubmap::Cloud::Ptr RollingSubmap::build_in_base_frame(
    const Eigen::Isometry3d& odom_T_current_base) const {
  Cloud::Ptr combined(new Cloud);
  combined->reserve(stored_point_count_);
  for (const Sample& sample : samples_) {
    const Eigen::Isometry3d current_base_T_sample_base =
        odom_T_current_base.inverse() * sample.odom_T_base;
    Cloud transformed;
    pcl::transformPointCloud(*sample.cloud, transformed,
                             current_base_T_sample_base.matrix().cast<float>());
    *combined += transformed;
  }

  if (combined->empty()) return combined;
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(config_.voxel_resolution, config_.voxel_resolution,
                    config_.voxel_resolution);
  voxel.setInputCloud(combined);
  voxel.filter(*filtered);
  if (config_.max_stored_points > 0 &&
      filtered->size() > config_.max_stored_points) {
    filtered->resize(config_.max_stored_points);
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
  }
  return filtered;
}

void RollingSubmap::clear() {
  samples_.clear();
  stored_point_count_ = 0;
}

void RollingSubmap::trim(double newest_stamp_s) {
  while (!samples_.empty() &&
         newest_stamp_s - samples_.front().stamp_s > config_.window_s) {
    stored_point_count_ -= samples_.front().cloud->size();
    samples_.pop_front();
  }
  while (samples_.size() > config_.max_samples && !samples_.empty()) {
    stored_point_count_ -= samples_.front().cloud->size();
    samples_.pop_front();
  }
  while (config_.max_stored_points > 0 && samples_.size() > 1 &&
         stored_point_count_ > config_.max_stored_points) {
    stored_point_count_ -= samples_.front().cloud->size();
    samples_.pop_front();
  }
}

}  // namespace static_livox_localization
