#include <gtest/gtest.h>
#include "static_livox_localization/registration.hpp"

using static_livox_localization::RegistrationConfig;
using static_livox_localization::register_cloud;

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

TEST(Registration, UsesSparserCloudForDensityImbalancedOverlap) {
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

  EXPECT_GT(result.inlier_ratio, 0.95);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
