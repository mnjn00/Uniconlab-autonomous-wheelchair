#pragma once

#include <string>

#include <Eigen/Geometry>

#include "static_livox_localization/registration.hpp"

namespace static_livox_localization {

enum class TrackingState {
  WAITING_INITIALIZATION,
  TRACKING,
  DEGRADED,
  LOST,
};

struct TrackingConfig {
  double max_fitness = 0.20;
  double min_inlier_ratio = 0.35;
  int min_source_points = 500;
  int min_target_points = 500;
  double max_prediction_translation_m = 1.0;
  double max_prediction_rotation_rad = 0.3490658503988659;
  double max_correction_translation_m = 0.30;
  double max_correction_rotation_rad = 0.08726646259971647;
  int degraded_after_failures = 1;
  double lost_after_s = 8.0;
  int recovery_confirmations = 2;
};

struct CorrectionDecision {
  bool accepted = false;
  std::string reason = "NOT_EVALUATED";
  double prediction_translation_m = 0.0;
  double prediction_rotation_rad = 0.0;
};

Eigen::Isometry3d compute_map_T_odom(
    const Eigen::Isometry3d& map_T_base_registered,
    const Eigen::Isometry3d& odom_T_base_at_stamp);

CorrectionDecision evaluate_correction(
    const RegistrationResult& registration,
    const Eigen::Isometry3d& predicted_map_T_base,
    const TrackingConfig& config);

// odom_T_base is what makes the limit mean anything: the step is clamped
// about the chair rather than about the odom origin. See the definition.
Eigen::Isometry3d limit_map_T_odom_step(
    const Eigen::Isometry3d& current_map_T_odom,
    const Eigen::Isometry3d& candidate_map_T_odom,
    const Eigen::Isometry3d& odom_T_base,
    const TrackingConfig& config);

bool tracking_motion_exceeds_threshold(
    const Eigen::Isometry3d& reference_odom_T_base,
    const Eigen::Isometry3d& current_odom_T_base,
    double minimum_translation_m,
    double minimum_rotation_rad);

const char* tracking_state_name(TrackingState state);

class TrackingStateMachine {
 public:
  explicit TrackingStateMachine(const TrackingConfig& config);

  void initialize(double stamp_s);
  TrackingState observe(bool accepted, double stamp_s);
  // Time in which no registration was attempted at all. LOST is declared on
  // how long it has been since the last accepted correction, and a parked
  // chair runs no registration by design - so without this, time spent
  // refusing to look counts as evidence of being lost.
  void note_unobserved(double stamp_s);
  TrackingState state() const { return state_; }
  int consecutive_failures() const { return consecutive_failures_; }

 private:
  TrackingConfig config_;
  TrackingState state_ = TrackingState::WAITING_INITIALIZATION;
  bool initialized_ = false;
  double last_accepted_stamp_s_ = 0.0;
  double unobserved_since_s_ = -1.0;
  int consecutive_failures_ = 0;
  int recovery_confirmations_ = 0;
};

}  // namespace static_livox_localization

