#pragma once

#include <string>

#include <Eigen/Geometry>

namespace static_livox_localization {

enum class AlignmentState {
  WAITING_INITIALIZATION,
  MANUAL_ALIGN,
  VERIFYING,
  TRACKING,
};

struct AlignmentConfig {
  int required_consistent_candidates = 3;
  double candidate_translation_tolerance_m = 0.20;
  double candidate_rotation_tolerance_rad = 0.05235987755982989;
};

struct ConsensusDecision {
  bool ready = false;
  int consistent_count = 0;
  std::string reason = "NOT_EVALUATED";
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
};

class AssistedAlignmentController {
 public:
  explicit AssistedAlignmentController(const AlignmentConfig& config);

  void on_seed();
  bool set_auto_correction(bool enabled);
  ConsensusDecision observe_candidate(const Eigen::Isometry3d& candidate);
  void observe_rejection();
  void begin_reacquisition();

  AlignmentState state() const { return state_; }
  bool auto_correction_enabled() const { return auto_correction_enabled_; }
  int consistent_count() const { return consistent_count_; }

 private:
  void clear_consensus();

  AlignmentConfig config_;
  AlignmentState state_ = AlignmentState::WAITING_INITIALIZATION;
  bool auto_correction_enabled_ = false;
  bool has_previous_candidate_ = false;
  int consistent_count_ = 0;
  Eigen::Isometry3d previous_candidate_ = Eigen::Isometry3d::Identity();
};

const char* alignment_state_name(AlignmentState state);

}  // namespace static_livox_localization
