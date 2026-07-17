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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
