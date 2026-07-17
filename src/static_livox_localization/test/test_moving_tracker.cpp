#include <cmath>

#include <gtest/gtest.h>

#include "static_livox_localization/moving_tracker.hpp"

namespace {

Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, 0.0);
  result.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

using static_livox_localization::CorrectionDecision;
using static_livox_localization::RegistrationResult;
using static_livox_localization::TrackingConfig;
using static_livox_localization::TrackingState;
using static_livox_localization::TrackingStateMachine;
using static_livox_localization::compute_map_T_odom;
using static_livox_localization::evaluate_correction;
using static_livox_localization::limit_map_T_odom_step;

TEST(MovingTracker, ComputesMapToOdomWithoutMovingMapFrame) {
  const Eigen::Isometry3d map_T_base = pose(12.0, -3.0, 0.4);
  const Eigen::Isometry3d odom_T_base = pose(2.0, 1.0, 0.1);

  const Eigen::Isometry3d map_T_odom =
      compute_map_T_odom(map_T_base, odom_T_base);

  EXPECT_TRUE((map_T_odom * odom_T_base)
                  .matrix()
                  .isApprox(map_T_base.matrix(), 1e-9));
}

TEST(MovingTracker, RejectsConvergedRegistrationWithTooFewInliers) {
  RegistrationResult registration;
  registration.converged = true;
  registration.fitness = 0.05;
  registration.inlier_ratio = 0.20;
  registration.source_points = 5000;
  registration.target_points = 10000;
  registration.map_T_base = pose(1.0, 0.0, 0.0);
  TrackingConfig config;
  config.min_inlier_ratio = 0.35;

  const CorrectionDecision decision =
      evaluate_correction(registration, pose(1.0, 0.0, 0.0), config);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "LOW_INLIER_RATIO");
}

TEST(MovingTracker, RejectsRegistrationFarFromOdometryPrediction) {
  RegistrationResult registration;
  registration.converged = true;
  registration.fitness = 0.05;
  registration.inlier_ratio = 0.80;
  registration.source_points = 5000;
  registration.target_points = 10000;
  registration.map_T_base = pose(4.0, 0.0, 0.0);
  TrackingConfig config;
  config.max_prediction_translation_m = 1.0;

  const CorrectionDecision decision =
      evaluate_correction(registration, pose(0.0, 0.0, 0.0), config);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "PREDICTION_TRANSLATION_JUMP");
}

TEST(MovingTracker, LimitsAcceptedMapToOdomCorrectionStep) {
  TrackingConfig config;
  config.max_correction_translation_m = 0.30;
  config.max_correction_rotation_rad = 5.0 * M_PI / 180.0;

  const Eigen::Isometry3d limited = limit_map_T_odom_step(
      Eigen::Isometry3d::Identity(), pose(1.0, 0.0, 0.5), config);

  EXPECT_NEAR(limited.translation().norm(), 0.30, 1e-9);
  EXPECT_NEAR(Eigen::AngleAxisd(limited.rotation()).angle(),
              config.max_correction_rotation_rad, 1e-9);
}

TEST(MovingTracker, DegradesThenLosesAndNeedsConfirmedRecovery) {
  TrackingConfig config;
  config.degraded_after_failures = 1;
  config.lost_after_s = 8.0;
  config.recovery_confirmations = 2;
  TrackingStateMachine machine(config);

  machine.initialize(10.0);
  EXPECT_EQ(machine.state(), TrackingState::TRACKING);

  machine.observe(false, 11.0);
  EXPECT_EQ(machine.state(), TrackingState::DEGRADED);

  machine.observe(false, 18.1);
  EXPECT_EQ(machine.state(), TrackingState::LOST);

  machine.observe(true, 19.0);
  EXPECT_EQ(machine.state(), TrackingState::LOST);
  machine.observe(true, 20.0);
  EXPECT_EQ(machine.state(), TrackingState::TRACKING);
}

TEST(MovingTracker, DoesNotUseSpeedAsCorrectionGate) {
  TrackingConfig config;
  RegistrationResult registration;
  registration.converged = true;
  registration.fitness = 0.01;
  registration.inlier_ratio = 0.90;
  registration.source_points = 2000;
  registration.target_points = 4000;
  registration.map_T_base = pose(0.2, 0.0, 0.0);

  const CorrectionDecision decision =
      evaluate_correction(registration, pose(0.0, 0.0, 0.0), config);

  EXPECT_TRUE(decision.accepted);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

