#include <gtest/gtest.h>

#include <cmath>

#include "wheelchair_lidar_localization/localization_core.hpp"

namespace wll = wheelchair_lidar_localization;
constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, 0.0);
  result.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

TEST(RegistrationGate, RequiresEveryMetric) {
  wll::AcceptanceLimits limits;
  wll::RegistrationMetrics metrics;
  metrics.converged = true;
  metrics.source_points = 1000;
  metrics.target_points = 2000;
  metrics.fitness = 0.10;
  metrics.inlier_ratio = 0.60;
  metrics.translation_correction_m = 0.10;
  metrics.rotation_correction_rad = 0.02;
  EXPECT_TRUE(wll::registrationAccepted(metrics, limits));
  metrics.inlier_ratio = 0.1;
  EXPECT_FALSE(wll::registrationAccepted(metrics, limits));
}

TEST(RegistrationGate, RejectsImpossibleNegativeDistances) {
  wll::AcceptanceLimits limits;
  wll::RegistrationMetrics metrics;
  metrics.converged = true;
  metrics.source_points = 1000;
  metrics.target_points = 2000;
  metrics.fitness = 0.10;
  metrics.inlier_ratio = 0.60;
  metrics.translation_correction_m = -0.01;
  metrics.rotation_correction_rad = 0.02;
  EXPECT_FALSE(wll::registrationAccepted(metrics, limits));
}

TEST(PoseConsensus, RejectsThreeDimensionalAliases) {
  wll::PoseConsensus consensus(3, 0.20, 3.0 * kPi / 180.0);
  EXPECT_FALSE(consensus.add(pose(1.00, 2.00, 0.10)));
  EXPECT_FALSE(consensus.add(pose(1.05, 2.02, 0.11)));
  EXPECT_FALSE(consensus.add(pose(8.00, -3.00, 2.8)));
  EXPECT_EQ(consensus.size(), 1u);
  EXPECT_FALSE(consensus.add(pose(8.03, -3.02, 2.79)));
  EXPECT_TRUE(consensus.add(pose(7.98, -2.97, 2.81)));
}

TEST(PoseConsensus, RejectsSameYawButUpsideDownPose) {
  wll::PoseConsensus consensus(2, 0.20, 3.0 * kPi / 180.0);
  const Eigen::Isometry3d normal = pose(1.0, 2.0, 0.1);
  Eigen::Isometry3d upside_down = normal;
  upside_down.linear() =
      normal.rotation() *
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_FALSE(consensus.add(normal));
  EXPECT_FALSE(consensus.add(upside_down));
  EXPECT_EQ(consensus.size(), 1u);
}
