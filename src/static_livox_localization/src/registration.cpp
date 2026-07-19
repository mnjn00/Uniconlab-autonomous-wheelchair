#include "static_livox_localization/registration.hpp"

#include <cmath>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace static_livox_localization {

RegistrationResult register_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& scan,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& map,
    const Eigen::Isometry3d& seed,
    const RegistrationConfig& config) {
  RegistrationResult result;
  if (!scan || !map || static_cast<int>(scan->size()) < config.min_points || map->empty()) return result;

  pcl::PointCloud<pcl::PointXYZI>::Ptr finite(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> kept;
  pcl::removeNaNFromPointCloud(*scan, *finite, kept);
  if (static_cast<int>(finite->size()) < config.min_points) return result;

  pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(config.voxel_resolution, config.voxel_resolution, config.voxel_resolution);
  voxel.setInputCloud(finite);
  voxel.filter(*source);
  if (static_cast<int>(source->size()) < config.min_points) return result;

  pcl::CropBox<pcl::PointXYZI> crop;
  crop.setInputCloud(map);
  const Eigen::Vector3d c = seed.translation();
  crop.setMin(Eigen::Vector4f(c.x() - config.roi_radius, c.y() - config.roi_radius,
                             c.z() - config.roi_z_half_extent, 1.0f));
  crop.setMax(Eigen::Vector4f(c.x() + config.roi_radius, c.y() + config.roi_radius,
                             c.z() + config.roi_z_half_extent, 1.0f));
  pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
  crop.filter(*target);
  result.source_points = static_cast<int>(source->size());
  result.target_points = static_cast<int>(target->size());
  if (static_cast<int>(target->size()) < config.min_points) return result;

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.setMaxCorrespondenceDistance(config.max_correspondence);
  gicp.setMaximumIterations(config.max_iterations);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-6);
  pcl::PointCloud<pcl::PointXYZI> aligned;
  gicp.align(aligned, seed.matrix().cast<float>());
  if (!gicp.hasConverged()) return result;

  result.map_T_base.matrix() = gicp.getFinalTransformation().cast<double>();
  result.fitness = gicp.getFitnessScore(config.max_correspondence);
  const Eigen::Isometry3d delta = seed.inverse() * result.map_T_base;
  const double translation = delta.translation().norm();
  const double rotation = Eigen::AngleAxisd(delta.rotation()).angle();

  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  int inliers = 0;
  std::vector<int> index(1);
  std::vector<float> distance(1);
  const double threshold2 = config.max_correspondence * config.max_correspondence;
  for (const auto& point : aligned.points) {
    if (tree.nearestKSearch(point, 1, index, distance) == 1 && distance[0] <= threshold2) ++inliers;
  }
  result.inlier_ratio = aligned.empty() ? 0.0 : static_cast<double>(inliers) / aligned.size();
  result.converged = std::isfinite(result.fitness) && result.fitness <= config.max_fitness &&
                     translation <= config.max_seed_translation &&
                     rotation <= config.max_seed_rotation_rad;
  return result;
}

}  // namespace static_livox_localization
