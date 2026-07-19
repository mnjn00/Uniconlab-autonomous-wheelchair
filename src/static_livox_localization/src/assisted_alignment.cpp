#include "static_livox_localization/assisted_alignment.hpp"

#include <algorithm>
#include <cmath>

namespace static_livox_localization {

AssistedAlignmentController::AssistedAlignmentController(
    const AlignmentConfig& config)
    : config_(config) {
  config_.required_consistent_candidates =
      std::max(1, config_.required_consistent_candidates);
  config_.candidate_translation_tolerance_m =
      std::max(0.0, config_.candidate_translation_tolerance_m);
  config_.candidate_rotation_tolerance_rad =
      std::max(0.0, config_.candidate_rotation_tolerance_rad);
}

void AssistedAlignmentController::clear_consensus() {
  has_previous_candidate_ = false;
  consistent_count_ = 0;
  previous_candidate_ = Eigen::Isometry3d::Identity();
}

void AssistedAlignmentController::on_seed() {
  clear_consensus();
  auto_correction_enabled_ = false;
  state_ = AlignmentState::MANUAL_ALIGN;
}

bool AssistedAlignmentController::set_auto_correction(bool enabled) {
  if (enabled && state_ == AlignmentState::WAITING_INITIALIZATION) return false;
  clear_consensus();
  auto_correction_enabled_ = enabled;
  state_ = enabled ? AlignmentState::VERIFYING : AlignmentState::MANUAL_ALIGN;
  return true;
}

ConsensusDecision AssistedAlignmentController::observe_candidate(
    const Eigen::Isometry3d& candidate) {
  ConsensusDecision decision;
  decision.candidate = candidate;
  if (!auto_correction_enabled_ ||
      state_ == AlignmentState::WAITING_INITIALIZATION ||
      state_ == AlignmentState::MANUAL_ALIGN) {
    decision.reason = "AUTO_CORRECTION_DISABLED";
    return decision;
  }

  bool consistent = true;
  if (has_previous_candidate_) {
    const Eigen::Isometry3d delta = previous_candidate_.inverse() * candidate;
    const double translation =
        std::hypot(delta.translation().x(), delta.translation().y());
    const double rotation =
        std::abs(std::atan2(delta.rotation()(1, 0),
                           delta.rotation()(0, 0)));
    consistent =
        translation <= config_.candidate_translation_tolerance_m &&
        rotation <= config_.candidate_rotation_tolerance_rad;
  }

  if (!has_previous_candidate_ || consistent) {
    ++consistent_count_;
    decision.reason = "CANDIDATE_ACCUMULATING";
  } else {
    consistent_count_ = 1;
    decision.reason = "CANDIDATE_INCONSISTENT";
  }
  previous_candidate_ = candidate;
  has_previous_candidate_ = true;
  decision.consistent_count = consistent_count_;

  if (consistent_count_ >= config_.required_consistent_candidates) {
    decision.ready = true;
    decision.reason = "CONSENSUS_READY";
    state_ = AlignmentState::TRACKING;
    clear_consensus();
  }
  return decision;
}

void AssistedAlignmentController::begin_reacquisition() {
  if (!auto_correction_enabled_ || state_ != AlignmentState::TRACKING) return;
  clear_consensus();
  state_ = AlignmentState::VERIFYING;
}

void AssistedAlignmentController::observe_rejection() {
  clear_consensus();
  if (auto_correction_enabled_ && state_ != AlignmentState::TRACKING) {
    state_ = AlignmentState::VERIFYING;
  }
}

const char* alignment_state_name(AlignmentState state) {
  switch (state) {
    case AlignmentState::WAITING_INITIALIZATION:
      return "WAITING_INITIALIZATION";
    case AlignmentState::MANUAL_ALIGN:
      return "MANUAL_ALIGN";
    case AlignmentState::VERIFYING:
      return "VERIFYING";
    case AlignmentState::TRACKING:
      return "TRACKING";
  }
  return "UNKNOWN";
}

}  // namespace static_livox_localization
