#pragma once

#include <cstddef>
#include <deque>
#include <vector>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace static_livox_localization {

struct RollingSubmapConfig {
  double window_s = 2.0;
  double voxel_resolution = 0.20;
  double max_stamp_skew_s = 0.10;
  std::size_t max_samples = 20;
  std::size_t max_stored_points = 120000;
  std::string expected_cloud_frame = "body";
};

// An axis-aligned box in the body frame, around something the tracker has
// watched move.
struct DynamicBox {
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
  Eigen::Vector3d half_extent = Eigen::Vector3d::Zero();
};

bool dynamic_box_within_limits(const DynamicBox& box,
                               double max_dimension_m,
                               double max_range_m);

// Drop returns that came off something moving, before they reach the submap.
//
// A pedestrian is not in the map, so GICP has nothing correct to match those
// returns to - and on 2026-08-09 it did not report a problem while being
// pulled: fitness held at 0.03 and inlier ratio ROSE to 0.993 while
// map_T_odom slid 0.504 m sideways over thirty seconds, 0.331 m of it in one
// correction as the person closed to 2 m. Every one of those corrections was
// accepted. No quality gate can catch this, because by every quality measure
// the alignment was excellent; it was excellent about the wrong solution.
//
// Filtered on the way IN rather than at build time: the submap accumulates
// over a 2 s window, so by the time it is built a walker at 1.4 m/s has laid
// a 2.8 m trail through it and the current box no longer covers the returns.
//
// max_dropped_fraction is retained in the API for diagnostics/config
// compatibility. Validated map-novel boxes are always excluded: restoring a
// crowd merely because it dominates a scan is the failure this filter exists
// to prevent.
std::size_t filter_dynamic_returns(
    pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std::vector<DynamicBox>& boxes,
    double margin_m,
    double max_dropped_fraction);

class RollingSubmap {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  explicit RollingSubmap(const RollingSubmapConfig& config);

  bool add_sample(const Cloud::ConstPtr& cloud, double cloud_stamp_s,
                  const Eigen::Isometry3d& odom_T_base,
                  double odom_stamp_s, const std::string& cloud_frame);

  Cloud::Ptr build_in_base_frame(
      const Eigen::Isometry3d& odom_T_current_base) const;

  void clear();
  std::size_t sample_count() const { return samples_.size(); }
  std::size_t stored_point_count() const { return stored_point_count_; }

 private:
  struct Sample {
    Cloud::Ptr cloud;
    double stamp_s = 0.0;
    Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  };

  void trim(double newest_stamp_s);

  RollingSubmapConfig config_;
  std::deque<Sample> samples_;
  std::size_t stored_point_count_ = 0;
};

}  // namespace static_livox_localization

