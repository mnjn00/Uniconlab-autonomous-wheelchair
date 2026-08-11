#include <gtest/gtest.h>
#include "static_livox_localization/registration.hpp"

using static_livox_localization::RegistrationConfig;
using static_livox_localization::register_cloud;
using static_livox_localization::registration_backend_available;

TEST(Registration, RejectsUnavailableBackendWithoutFallback) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  RegistrationConfig config;
  config.backend = "missing_backend";
  const auto result = register_cloud(
      scan, map, Eigen::Isometry3d::Identity(), config);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.backend, "missing_backend");
  EXPECT_EQ(result.error, "REGISTRATION_BACKEND_UNAVAILABLE");
}

TEST(Registration, RejectsInsufficientScan) {
  auto scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->resize(10);
  map->resize(1000);
  EXPECT_FALSE(register_cloud(scan, map, Eigen::Isometry3d::Identity(), RegistrationConfig()).converged);
}

TEST(Registration, AlignsStructuredIdentityCloud) {
  auto scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int x = 0; x < 20; ++x) for (int y = 0; y < 20; ++y) for (int z = 0; z < 3; ++z) {
    pcl::PointXYZI p; p.x = x * 0.25f; p.y = y * 0.25f; p.z = z * 0.35f + 0.01f * x; p.intensity = x + y;
    scan->push_back(p);
  }
  auto map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*scan);
  RegistrationConfig config; config.min_points = 300; config.max_fitness = 0.01;
  const auto result = register_cloud(scan, map, Eigen::Isometry3d::Identity(), config);
  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.fitness, 1e-4);
}

TEST(Registration, CudaBackendAlignsStructuredIdentityCloudWhenBuilt) {
  if (!registration_backend_available("fast_vgicp_cuda")) return;

  auto scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      for (int z = 0; z < 3; ++z) {
        pcl::PointXYZI point;
        point.x = x * 0.25f;
        point.y = y * 0.25f;
        point.z = z * 0.35f + 0.01f * x;
        point.intensity = x + y;
        scan->push_back(point);
      }
    }
  }
  auto map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*scan);
  RegistrationConfig config;
  config.backend = "fast_vgicp_cuda";
  config.min_points = 300;
  // VGICP optimizes voxel distributions rather than the point-wise GICP
  // objective, so an identity input is not expected to reproduce PCL's
  // near-zero score on this synthetic lattice. This is a kernel/contract
  // smoke test; route-specific score equivalence belongs to rosbag replay.
  config.max_fitness = 0.10;
  const auto result = register_cloud(
      scan, map, Eigen::Isometry3d::Identity(), config);
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.fitness, 0.10);
  EXPECT_EQ(result.backend, "fast_vgicp_cuda");
  EXPECT_GT(result.elapsed_ms, 0.0);
}

TEST(Registration, ScoresTheScanAgainstTheMapWhateverTheDensities) {
  auto sparse_map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto dense_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int x = 0; x < 56; ++x) {
    for (int y = 0; y < 56; ++y) {
      for (int z = 0; z < 3; ++z) {
        pcl::PointXYZI p;
        p.x = x * 0.08f;
        p.y = y * 0.08f;
        p.z = z * 0.40f + 0.002f * x;
        dense_scan->push_back(p);
      }
    }
  }
  for (int x = 0; x < 12; ++x) {
    for (int y = 0; y < 12; ++y) {
      for (int z = 0; z < 3; ++z) {
        pcl::PointXYZI p;
        p.x = x * 0.40f;
        p.y = y * 0.40f;
        p.z = z * 0.40f + 0.01f * x;
        sparse_map->push_back(p);
      }
    }
  }

  RegistrationConfig config;
  config.voxel_resolution = 0.05;
  config.min_points = 300;
  config.max_correspondence = 0.12;
  config.max_fitness = 0.02;
  const auto result = register_cloud(
      dense_scan, sparse_map, Eigen::Isometry3d::Identity(), config);

  // The ratio answers "how much of what I see is mapped", always in that
  // direction. defd0d2 fixed it there deliberately: scoring whichever cloud
  // happened to be sparser flipped the question as the clouds changed size,
  // and only this direction measures localization - the other one falls with
  // occlusion and with how richly the map was built.
  //
  // So the honest expectation for a 0.40 m map, a 0.08 m scan and a 0.12 m
  // correspondence radius is the fraction of scan points lying near a
  // lattice node, about (0.24/0.40)^2 = 0.36 - not the 0.95 this asserted
  // while it was still named for the behaviour defd0d2 removed. Perfectly
  // aligned, a sparse map simply cannot explain most of a dense scan.
  EXPECT_GT(result.inlier_ratio, 0.30);
  EXPECT_LT(result.inlier_ratio, 0.45);
  // Not asserting fitness against config.max_fitness here: a 0.40 m map
  // cannot put a neighbour close to every point of a 0.08 m scan, so the
  // mean squared correspondence distance sits above that limit (0.025)
  // however well the clouds are aligned. Density, not misalignment.
}

TEST(Registration, AnIterationLimitedAlignmentIsStillMeasured) {
  // hasConverged() false means the optimiser used its whole iteration budget
  // without the transform settling under transformationEpsilon 1e-6 - not
  // that the alignment is wrong. Returning early on it published fitness 1e9
  // and inlier 0.0 for alignments that were correct, and the follower read
  // that as a lost fix: 53 NOT_CONVERGED events in one 1413 s run, and a
  // 150 s stop with the map visibly aligned in RViz. One iteration cannot
  // meet a 1e-6 epsilon, so this exercises exactly that path.
  auto scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int x = 0; x < 20; ++x) for (int y = 0; y < 20; ++y) for (int z = 0; z < 3; ++z) {
    pcl::PointXYZI p; p.x = x * 0.25f; p.y = y * 0.25f;
    p.z = z * 0.35f + 0.01f * x; p.intensity = x + y;
    scan->push_back(p);
  }
  auto map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*scan);
  RegistrationConfig config;
  config.min_points = 300;
  config.max_iterations = 1;
  const auto result = register_cloud(
      scan, map, Eigen::Isometry3d::Identity(), config);
  EXPECT_LT(result.fitness, 1e8) << "fitness was never computed";
  EXPECT_GT(result.inlier_ratio, 0.0) << "inlier ratio was never computed";
  EXPECT_GT(result.source_points, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
