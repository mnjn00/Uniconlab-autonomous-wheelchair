#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace static_livox_localization {

struct RegistrationConfig {
  double voxel_resolution = 0.20;
  double roi_radius = 20.0;
  double roi_z_half_extent = 5.0;
  double max_correspondence = 1.0;
  int max_iterations = 64;
  int min_points = 500;
  double max_fitness = 0.20;
  double max_seed_translation = 3.0;
  double max_seed_rotation_rad = 0.5235987755982988;
};

struct RegistrationResult {
  Eigen::Isometry3d map_T_base = Eigen::Isometry3d::Identity();
  double fitness = 1e9;
  double inlier_ratio = 0.0;
  bool converged = false;
  int source_points = 0;
  int target_points = 0;
};

RegistrationResult register_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& scan,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& map,
    const Eigen::Isometry3d& seed,
    const RegistrationConfig& config);

}  // namespace static_livox_localization
