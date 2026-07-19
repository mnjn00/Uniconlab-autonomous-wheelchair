#pragma once

#include <cstddef>
#include <deque>
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

