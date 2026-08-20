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
using static_livox_localization::tracking_motion_exceeds_threshold;

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

TEST(MovingTracker, IgnoresVerticalDriftForPlanarPredictionGate) {
  RegistrationResult registration;
  registration.converged = true;
  registration.fitness = 0.05;
  registration.inlier_ratio = 0.80;
  registration.source_points = 5000;
  registration.target_points = 10000;
  registration.map_T_base = Eigen::Isometry3d::Identity();
  registration.map_T_base.translation().z() = 2.0;
  TrackingConfig config;
  config.max_prediction_translation_m = 0.50;

  const CorrectionDecision decision = evaluate_correction(
      registration, Eigen::Isometry3d::Identity(), config);

  EXPECT_TRUE(decision.accepted);
  EXPECT_NEAR(decision.prediction_translation_m, 0.0, 1e-9);
  EXPECT_NEAR(decision.prediction_rotation_rad, 0.0, 1e-9);
}

TEST(MovingTracker, LimitsAcceptedMapToOdomCorrectionStep) {
  TrackingConfig config;
  config.max_correction_translation_m = 0.30;
  config.max_correction_rotation_rad = 5.0 * M_PI / 180.0;

  const Eigen::Isometry3d limited = limit_map_T_odom_step(
      Eigen::Isometry3d::Identity(), pose(1.0, 0.0, 0.5),
      Eigen::Isometry3d::Identity(), config);

  EXPECT_NEAR(limited.translation().norm(), 0.30, 1e-9);
  EXPECT_NEAR(Eigen::AngleAxisd(limited.rotation()).angle(),
              config.max_correction_rotation_rad, 1e-9);
}

// A yaw-only correction about the odom origin is what put the chair 1.92 m
// sideways on 2026-08-20 while every gate read nominal. The clamp has to bound
// what the chair does, not what the transform says.
TEST(MovingTracker, BoundsChairDisplacementFarFromTheOdomOrigin) {
  TrackingConfig config;
  config.max_correction_translation_m = 0.20;
  config.max_correction_rotation_rad = 2.0 * M_PI / 180.0;

  const double r = 148.0;
  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  odom_T_base.translation() = Eigen::Vector3d(r, 0.0, 0.0);

  // 0.53 deg about the odom origin - the correction actually applied that day.
  const double yaw = 0.53 * M_PI / 180.0;
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
  candidate.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d limited =
      limit_map_T_odom_step(current, candidate, odom_T_base, config);

  const Eigen::Vector3d before = (current * odom_T_base).translation();
  const Eigen::Vector3d after = (limited * odom_T_base).translation();
  const double unclamped = (candidate * odom_T_base).translation().x() - before.x();
  (void)unclamped;

  // Left alone this is r * yaw = 1.37 m.
  EXPECT_GT(((candidate * odom_T_base).translation() - before).norm(), 1.3);
  EXPECT_LE((after - before).norm(),
            config.max_correction_translation_m + 1e-9);
}

TEST(MovingTracker, RequiresRealMotionBeforeAnotherTrackingCorrection) {
  const Eigen::Isometry3d reference = pose(2.0, 3.0, 0.2);

  EXPECT_FALSE(tracking_motion_exceeds_threshold(
      reference, pose(2.007, 3.0, 0.205), 0.10, 2.0 * M_PI / 180.0));
  EXPECT_TRUE(tracking_motion_exceeds_threshold(
      reference, pose(2.11, 3.0, 0.205), 0.10, 2.0 * M_PI / 180.0));
  EXPECT_TRUE(tracking_motion_exceeds_threshold(
      reference, pose(2.0, 3.0, 0.25), 0.10, 2.0 * M_PI / 180.0));
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

TEST(MovingTracker, TimeNobodyLookedAtIsNotEvidenceOfBeingLost) {
  // Corrections are suppressed while the chair is parked, so no registration
  // runs and observe() is never called. LOST is declared on elapsed time
  // since the last ACCEPTED correction, and lost_after_s is 8 s - so any
  // hold longer than that used to end in LOST on the first imperfect
  // registration after the chair moved again, with no chance to prove
  // otherwise. On 2026-08-09 holds of 36, 46, 137 and 266 s all qualified.
  TrackingConfig config;
  config.degraded_after_failures = 1;
  config.lost_after_s = 8.0;
  TrackingStateMachine machine(config);
  machine.initialize(0.0);
  EXPECT_EQ(machine.observe(true, 1.0), TrackingState::TRACKING);

  for (double t = 1.5; t < 60.0; t += 0.5) machine.note_unobserved(t);

  // One failure after a 59 s parked stretch is a failure, not a loss.
  EXPECT_EQ(machine.observe(false, 60.0), TrackingState::DEGRADED);
  // And the clock still works: genuine unobserved-free time does lose it.
  EXPECT_EQ(machine.observe(false, 70.0), TrackingState::LOST);
}

TEST(MovingTracker, DiscountingUnobservedTimeDoesNotResetTheFailureCount) {
  TrackingConfig config;
  config.degraded_after_failures = 2;
  config.lost_after_s = 100.0;
  TrackingStateMachine machine(config);
  machine.initialize(0.0);
  machine.observe(true, 1.0);
  EXPECT_EQ(machine.observe(false, 2.0), TrackingState::TRACKING);
  machine.note_unobserved(3.0);
  // The parked stretch says nothing about the failure that preceded it.
  EXPECT_EQ(machine.observe(false, 30.0), TrackingState::DEGRADED);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

