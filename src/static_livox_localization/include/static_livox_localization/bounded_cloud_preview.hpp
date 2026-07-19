#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace static_livox_localization {

struct PreviewConfig {
  double voxel_resolution = 0.30;
  double max_rate_hz = 5.0;
};

class BoundedCloudPreview {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  explicit BoundedCloudPreview(const PreviewConfig& config);

  bool should_publish(double stamp_s);
  Cloud::Ptr downsample(const Cloud::ConstPtr& input) const;

 private:
  PreviewConfig config_;
  bool has_last_publish_stamp_ = false;
  double last_publish_stamp_s_ = 0.0;
};

}  // namespace static_livox_localization
