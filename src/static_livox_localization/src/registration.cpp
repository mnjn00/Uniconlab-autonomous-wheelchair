#include "static_livox_localization/registration.hpp"

#include <cmath>
#include <chrono>
#include <exception>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#ifdef STATIC_LIVOX_HAS_FAST_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace static_livox_localization {

bool registration_backend_available(const std::string& backend) {
  if (backend == "pcl_gicp") return true;
#ifdef STATIC_LIVOX_HAS_FAST_VGICP_CUDA
  if (backend == "fast_vgicp_cuda") return true;
#endif
  return false;
}

RegistrationResult register_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& scan,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& map,
    const Eigen::Isometry3d& seed,
    const RegistrationConfig& config) {
  const auto started = std::chrono::steady_clock::now();
  RegistrationResult result;
  result.backend = config.backend;
  const auto finish = [&]() {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return result;
  };
  if (!registration_backend_available(config.backend)) {
    result.error = "REGISTRATION_BACKEND_UNAVAILABLE";
    return finish();
  }
  if (!scan || !map || static_cast<int>(scan->size()) < config.min_points || map->empty()) return finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr finite(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> kept;
  pcl::removeNaNFromPointCloud(*scan, *finite, kept);
  if (static_cast<int>(finite->size()) < config.min_points) return finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(config.voxel_resolution, config.voxel_resolution, config.voxel_resolution);
  voxel.setInputCloud(finite);
  voxel.filter(*source);
  if (static_cast<int>(source->size()) < config.min_points) return finish();

  pcl::CropBox<pcl::PointXYZI> crop;
  crop.setInputCloud(map);
  const Eigen::Vector3d c = seed.translation();
  const Eigen::Vector3f roi_min(c.x() - config.roi_radius,
                                c.y() - config.roi_radius,
                                c.z() - config.roi_z_half_extent);
  const Eigen::Vector3f roi_max(c.x() + config.roi_radius,
                                c.y() + config.roi_radius,
                                c.z() + config.roi_z_half_extent);
  crop.setMin(Eigen::Vector4f(roi_min.x(), roi_min.y(), roi_min.z(), 1.0f));
  crop.setMax(Eigen::Vector4f(roi_max.x(), roi_max.y(), roi_max.z(), 1.0f));
  pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
  crop.filter(*target);
  result.source_points = static_cast<int>(source->size());
  result.target_points = static_cast<int>(target->size());
  if (static_cast<int>(target->size()) < config.min_points) return finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
  try {
    if (config.backend == "pcl_gicp") {
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
      gicp.setInputSource(source);
      gicp.setInputTarget(target);
      gicp.setMaxCorrespondenceDistance(config.max_correspondence);
      gicp.setMaximumIterations(config.max_iterations);
      gicp.setTransformationEpsilon(config.transformation_epsilon);
      gicp.setEuclideanFitnessEpsilon(1e-6);
      // Reciprocal correspondences: a match is kept only if each point is
      // the other's nearest neighbour. This rejects one-way matches
      // typical of dynamic objects, which are near map structure but not
      // reciprocally near.
      gicp.setUseReciprocalCorrespondences(true);
      gicp.align(*aligned, seed.matrix().cast<float>());
      result.epsilon_met = gicp.hasConverged();
      final_transformation = gicp.getFinalTransformation();
      result.fitness = gicp.getFitnessScore(config.max_correspondence);
    }
#ifdef STATIC_LIVOX_HAS_FAST_VGICP_CUDA
    else if (config.backend == "fast_vgicp_cuda") {
      fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI> gicp;
      gicp.setInputSource(source);
      gicp.setInputTarget(target);
      gicp.setResolution(config.voxel_resolution);
      gicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      // Unmeasured, and the first place to look at the cycle time: the CUDA
      // backend is doing its nearest-neighbour search on the CPU. On
      // 2026-08-16 a registration took 370 ms against a 10 Hz input, so the
      // localizer handled about 2.7 of every 10 clouds, fell behind, and
      // ended up unable to pair the cloud it held with an odometry sample -
      // CLOUD_ODOMETRY_TIME_MISMATCH, which it did not recover from without
      // a restart. Where that 370 ms goes has not been profiled. fast_gicp
      // also offers GPU_BRUTEFORCE and GPU_RBF_KERNEL; switching needs a GPU
      // run and a fitness comparison, so it is left alone rather than
      // swapped blind.
      gicp.setNearestNeighborSearchMethod(
          fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
      gicp.setMaxCorrespondenceDistance(config.max_correspondence);
      gicp.setMaximumIterations(config.max_iterations);
      gicp.setTransformationEpsilon(config.transformation_epsilon);
      gicp.setEuclideanFitnessEpsilon(1e-6);
      gicp.align(*aligned, seed.matrix().cast<float>());
      result.epsilon_met = gicp.hasConverged();
      final_transformation = gicp.getFinalTransformation();
      result.fitness = gicp.getFitnessScore(config.max_correspondence);
    }
#endif
  } catch (const std::exception& exception) {
    result.error = std::string("REGISTRATION_BACKEND_ERROR: ") + exception.what();
    return finish();
  } catch (...) {
    result.error = "REGISTRATION_BACKEND_ERROR: unknown exception";
    return finish();
  }

  result.map_T_base.matrix() = final_transformation.cast<double>();
  const Eigen::Isometry3d delta = seed.inverse() * result.map_T_base;
  const double translation = delta.translation().norm();
  const double rotation = Eigen::AngleAxisd(delta.rotation()).angle();

  // Score only the scan points the map could ever explain. The scan runs
  // out to the lidar's full range while the target is cropped to the ROI
  // box, so counting the points beyond that box as outliers measured how
  // far the lidar sees rather than how well the pose fits: an open street
  // scored worse than a narrow one at the same alignment. Measured on the
  // chair, this held the ratio near 0.18 while the same points scored
  // 0.87 once restricted to the ROI, and it sank further as the chair
  // drove into open ground.
  //
  // The direction is fixed too. Taking whichever cloud happened to be
  // smaller flipped the question between "how much of what I see is
  // mapped" and "how much of the map do I see" as the clouds changed
  // size; only the former measures localization, since the latter falls
  // with occlusion and with map richness.
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  int inliers = 0;
  int scored = 0;
  std::vector<int> index(1);
  std::vector<float> distance(1);
  const double threshold2 = config.max_correspondence * config.max_correspondence;
  for (const auto& point : aligned->points) {
    if (point.x < roi_min.x() || point.x > roi_max.x() ||
        point.y < roi_min.y() || point.y > roi_max.y() ||
        point.z < roi_min.z() || point.z > roi_max.z()) {
      continue;
    }
    ++scored;
    if (tree.nearestKSearch(point, 1, index, distance) == 1 && distance[0] <= threshold2) ++inliers;
  }
  result.inlier_ratio =
      scored == 0 ? 0.0 : static_cast<double>(inliers) / scored;
  result.converged = std::isfinite(result.fitness) && result.fitness <= config.max_fitness &&
                     translation <= config.max_seed_translation &&
                     rotation <= config.max_seed_rotation_rad;
  return finish();
}

}  // namespace static_livox_localization
