#include <gtest/gtest.h>

#include <cmath>

#include "static_livox_localization/assisted_alignment.hpp"

namespace static_livox_localization {

TEST(AssistedAlignment, RequiresSeedBeforeAutomaticCorrection) {
  AssistedAlignmentController controller(AlignmentConfig{});

  EXPECT_FALSE(controller.set_auto_correction(true));
  EXPECT_EQ(controller.state(), AlignmentState::WAITING_INITIALIZATION);
}

TEST(AssistedAlignment, SeedForcesManualAndDisablesAutomaticCorrection) {
  AssistedAlignmentController controller(AlignmentConfig{});
  controller.on_seed();

  EXPECT_EQ(controller.state(), AlignmentState::MANUAL_ALIGN);
  EXPECT_FALSE(controller.auto_correction_enabled());
}

TEST(AssistedAlignment, ThreeConsistentCandidatesEnterTracking) {
  AlignmentConfig config;
  config.required_consistent_candidates = 3;
  config.candidate_translation_tolerance_m = 0.20;
  config.candidate_rotation_tolerance_rad = 3.0 * M_PI / 180.0;
  AssistedAlignmentController controller(config);
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));

  EXPECT_FALSE(controller.observe_candidate(Eigen::Isometry3d::Identity()).ready);
  EXPECT_FALSE(controller.observe_candidate(Eigen::Isometry3d::Identity()).ready);
  const ConsensusDecision decision =
      controller.observe_candidate(Eigen::Isometry3d::Identity());

  EXPECT_TRUE(decision.ready);
  EXPECT_EQ(decision.consistent_count, 3);
  EXPECT_EQ(controller.state(), AlignmentState::TRACKING);
}

TEST(AssistedAlignment, InconsistentCandidateRestartsConsensus) {
  AlignmentConfig config;
  config.required_consistent_candidates = 3;
  config.candidate_translation_tolerance_m = 0.20;
  AssistedAlignmentController controller(config);
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));
  controller.observe_candidate(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d jump = Eigen::Isometry3d::Identity();
  jump.translation().x() = 1.0;

  const ConsensusDecision decision = controller.observe_candidate(jump);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.consistent_count, 1);
  EXPECT_EQ(decision.reason, "CANDIDATE_INCONSISTENT");
}

TEST(AssistedAlignment, IgnoresVerticalDriftForPlanarConsensus) {
  AlignmentConfig config;
  config.required_consistent_candidates = 3;
  config.candidate_translation_tolerance_m = 0.20;
  config.candidate_rotation_tolerance_rad = 3.0 * M_PI / 180.0;
  AssistedAlignmentController controller(config);
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));

  Eigen::Isometry3d first = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d second = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d third = Eigen::Isometry3d::Identity();
  second.translation().z() = 1.0;
  third.translation().z() = 2.0;

  EXPECT_FALSE(controller.observe_candidate(first).ready);
  EXPECT_FALSE(controller.observe_candidate(second).ready);
  const ConsensusDecision decision = controller.observe_candidate(third);

  EXPECT_TRUE(decision.ready);
  EXPECT_EQ(controller.state(), AlignmentState::TRACKING);
}

TEST(AssistedAlignment, RejectionClearsPendingConsensus) {
  AssistedAlignmentController controller(AlignmentConfig{});
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));
  controller.observe_candidate(Eigen::Isometry3d::Identity());

  controller.observe_rejection();

  EXPECT_EQ(controller.consistent_count(), 0);
  EXPECT_EQ(controller.state(), AlignmentState::VERIFYING);
}

TEST(AssistedAlignment, LostTrackingReentersVerifyingForReacquisition) {
  AlignmentConfig config;
  config.required_consistent_candidates = 1;
  AssistedAlignmentController controller(config);
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));
  ASSERT_TRUE(controller.observe_candidate(Eigen::Isometry3d::Identity()).ready);
  ASSERT_EQ(controller.state(), AlignmentState::TRACKING);

  controller.begin_reacquisition();

  EXPECT_EQ(controller.state(), AlignmentState::VERIFYING);
  EXPECT_TRUE(controller.auto_correction_enabled());
  EXPECT_EQ(controller.consistent_count(), 0);
  EXPECT_TRUE(controller.observe_candidate(Eigen::Isometry3d::Identity()).ready);
  EXPECT_EQ(controller.state(), AlignmentState::TRACKING);
}

TEST(AssistedAlignment, ReacquisitionRequiresActiveTracking) {
  AssistedAlignmentController controller(AlignmentConfig{});
  controller.on_seed();

  controller.begin_reacquisition();

  EXPECT_EQ(controller.state(), AlignmentState::MANUAL_ALIGN);
}

TEST(AssistedAlignment, NewSeedReturnsTrackingControllerToManual) {
  AlignmentConfig config;
  config.required_consistent_candidates = 1;
  AssistedAlignmentController controller(config);
  controller.on_seed();
  ASSERT_TRUE(controller.set_auto_correction(true));
  ASSERT_TRUE(controller.observe_candidate(Eigen::Isometry3d::Identity()).ready);

  controller.on_seed();

  EXPECT_EQ(controller.state(), AlignmentState::MANUAL_ALIGN);
  EXPECT_FALSE(controller.auto_correction_enabled());
}

}  // namespace static_livox_localization

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
