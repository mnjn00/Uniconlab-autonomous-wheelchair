#include <cmath>

#include <gtest/gtest.h>

#include "static_livox_localization/rolling_submap.hpp"

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Cloud::Ptr cloud_with_point(double x, double y) {
  Cloud::Ptr cloud(new Cloud);
  pcl::PointXYZI point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = 0.0f;
  point.intensity = 1.0f;
  cloud->push_back(point);
  return cloud;
}

Cloud::Ptr cloud_with_count(int count) {
  Cloud::Ptr cloud(new Cloud);
  for (int i = 0; i < count; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(i) * 0.1f;
    point.y = static_cast<float>(i % 3) * 0.2f;
    point.z = 0.0f;
    cloud->push_back(point);
  }
  return cloud;
}

Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, 0.0);
  result.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

using static_livox_localization::RollingSubmap;
using static_livox_localization::RollingSubmapConfig;

TEST(RollingSubmap, CompensatesTranslationIntoCurrentBaseFrame) {
  RollingSubmapConfig config;
  config.voxel_resolution = 0.01;
  RollingSubmap submap(config);

  ASSERT_TRUE(submap.add_sample(cloud_with_point(5.0, 0.0), 0.0,
                                pose(0.0, 0.0, 0.0), 0.0, "body"));
  ASSERT_TRUE(submap.add_sample(cloud_with_point(4.0, 0.0), 1.0,
                                pose(1.0, 0.0, 0.0), 1.0, "body"));

  const Cloud::Ptr result =
      submap.build_in_base_frame(pose(1.0, 0.0, 0.0));

  ASSERT_EQ(result->size(), 1u);
  EXPECT_NEAR(result->front().x, 4.0, 1e-3);
  EXPECT_NEAR(result->front().y, 0.0, 1e-3);
}

TEST(RollingSubmap, CompensatesYawIntoCurrentBaseFrame) {
  RollingSubmapConfig config;
  config.voxel_resolution = 0.01;
  RollingSubmap submap(config);
  const double half_pi = M_PI / 2.0;

  ASSERT_TRUE(submap.add_sample(cloud_with_point(5.0, 0.0), 0.0,
                                pose(0.0, 0.0, 0.0), 0.0, "body"));
  ASSERT_TRUE(submap.add_sample(cloud_with_point(0.0, -5.0), 1.0,
                                pose(0.0, 0.0, half_pi), 1.0, "body"));

  const Cloud::Ptr result =
      submap.build_in_base_frame(pose(0.0, 0.0, half_pi));

  ASSERT_EQ(result->size(), 1u);
  EXPECT_NEAR(result->front().x, 0.0, 1e-3);
  EXPECT_NEAR(result->front().y, -5.0, 1e-3);
}

TEST(RollingSubmap, RemovesSamplesOlderThanRollingWindow) {
  RollingSubmapConfig config;
  config.window_s = 2.0;
  RollingSubmap submap(config);

  EXPECT_TRUE(submap.add_sample(cloud_with_point(1.0, 0.0), 0.0,
                                pose(0.0, 0.0, 0.0), 0.0, "body"));
  EXPECT_TRUE(submap.add_sample(cloud_with_point(1.0, 0.0), 1.0,
                                pose(0.0, 0.0, 0.0), 1.0, "body"));
  EXPECT_TRUE(submap.add_sample(cloud_with_point(1.0, 0.0), 3.0,
                                pose(0.0, 0.0, 0.0), 3.0, "body"));

  EXPECT_EQ(submap.sample_count(), 2u);
}

TEST(RollingSubmap, RejectsTimestampSkewAndUnexpectedFrame) {
  RollingSubmapConfig config;
  config.max_stamp_skew_s = 0.10;
  config.expected_cloud_frame = "body";
  RollingSubmap submap(config);

  EXPECT_FALSE(submap.add_sample(cloud_with_point(1.0, 0.0), 1.0,
                                 pose(0.0, 0.0, 0.0), 1.2, "body"));
  EXPECT_FALSE(submap.add_sample(cloud_with_point(1.0, 0.0), 1.0,
                                 pose(0.0, 0.0, 0.0), 1.0,
                                 "camera_init"));
  EXPECT_EQ(submap.sample_count(), 0u);
}

TEST(RollingSubmap, BoundsSamplesAndStoredPoints) {
  RollingSubmapConfig config;
  config.max_samples = 2;
  config.max_stored_points = 15;
  RollingSubmap submap(config);

  EXPECT_TRUE(submap.add_sample(cloud_with_count(10), 0.0,
                                pose(0.0, 0.0, 0.0), 0.0, "body"));
  EXPECT_TRUE(submap.add_sample(cloud_with_count(10), 1.0,
                                pose(0.0, 0.0, 0.0), 1.0, "body"));

  EXPECT_EQ(submap.sample_count(), 1u);
  EXPECT_LE(submap.stored_point_count(), 15u);
}


namespace {
pcl::PointCloud<pcl::PointXYZI> grid_cloud(int n) {
  pcl::PointCloud<pcl::PointXYZI> c;
  for (int i = 0; i < n; ++i) {
    pcl::PointXYZI p;
    p.x = 0.1f * i; p.y = 0.0f; p.z = 0.0f; p.intensity = 1.0f;
    c.push_back(p);
  }
  return c;
}
}  // namespace

TEST(RollingSubmap, DropsReturnsInsideAMovingBox) {
  auto cloud = grid_cloud(100);            // x = 0.0 .. 9.9
  static_livox_localization::DynamicBox box;
  box.centre = Eigen::Vector3d(2.0, 0.0, 0.0);
  box.half_extent = Eigen::Vector3d(0.5, 0.5, 0.5);
  const std::size_t dropped =
      static_livox_localization::filter_dynamic_returns(cloud, {box}, 0.0, 0.5);
  EXPECT_GT(dropped, 0u);
  EXPECT_EQ(cloud.size(), 100u - dropped);
  for (const auto& p : cloud.points) {
    EXPECT_FALSE(std::abs(p.x - 2.0) <= 0.5 && std::abs(p.y) <= 0.5);
  }
}

TEST(RollingSubmap, CrowdDominatedScanIsStillExcluded) {
  // A validated map-novel box is never registration structure. Restoring it
  // because the crowd happens to dominate the scan recreates the field bug.
  auto cloud = grid_cloud(100);
  static_livox_localization::DynamicBox box;
  box.centre = Eigen::Vector3d(5.0, 0.0, 0.0);
  box.half_extent = Eigen::Vector3d(9.0, 1.0, 1.0);
  const std::size_t dropped =
      static_livox_localization::filter_dynamic_returns(cloud, {box}, 0.0, 0.25);
  EXPECT_EQ(dropped, 100u);
  EXPECT_TRUE(cloud.empty());
}

TEST(RollingSubmap, NoBoxesLeavesTheScanExactlyAsItWas) {
  auto cloud = grid_cloud(50);
  EXPECT_EQ(static_livox_localization::filter_dynamic_returns(cloud, {}, 0.2, 0.25), 0u);
  EXPECT_EQ(cloud.size(), 50u);
}

TEST(RollingSubmap, TheMarginWidensTheBox) {
  auto a = grid_cloud(100);
  auto b = grid_cloud(100);
  static_livox_localization::DynamicBox box;
  box.centre = Eigen::Vector3d(2.0, 0.0, 0.0);
  box.half_extent = Eigen::Vector3d(0.2, 0.5, 0.5);
  const std::size_t tight = static_livox_localization::filter_dynamic_returns(a, {box}, 0.0, 0.9);
  const std::size_t wide = static_livox_localization::filter_dynamic_returns(b, {box}, 0.5, 0.9);
  EXPECT_GT(wide, tight);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

