#include "static_livox_localization/moving_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace static_livox_localization {
namespace {

double rotation_angle(const Eigen::Matrix3d& rotation) {
  return std::abs(Eigen::AngleAxisd(rotation).angle());
}

double planar_translation(const Eigen::Vector3d& translation) {
  return std::hypot(translation.x(), translation.y());
}

double yaw_angle(const Eigen::Matrix3d& rotation) {
  return std::abs(std::atan2(rotation(1, 0), rotation(0, 0)));
}

}  // namespace

Eigen::Isometry3d compute_map_T_odom(
    const Eigen::Isometry3d& map_T_base_registered,
    const Eigen::Isometry3d& odom_T_base_at_stamp) {
  return map_T_base_registered * odom_T_base_at_stamp.inverse();
}

CorrectionDecision evaluate_correction(
    const RegistrationResult& registration,
    const Eigen::Isometry3d& predicted_map_T_base,
    const TrackingConfig& config) {
  CorrectionDecision decision;
  if (!registration.converged) {
    decision.reason = "NOT_CONVERGED";
    return decision;
  }
  if (!std::isfinite(registration.fitness) ||
      registration.fitness > config.max_fitness) {
    decision.reason = "HIGH_FITNESS";
    return decision;
  }
  if (registration.source_points < config.min_source_points) {
    decision.reason = "INSUFFICIENT_SOURCE_POINTS";
    return decision;
  }
  if (registration.target_points < config.min_target_points) {
    decision.reason = "INSUFFICIENT_TARGET_POINTS";
    return decision;
  }
  if (!std::isfinite(registration.inlier_ratio) ||
      registration.inlier_ratio < config.min_inlier_ratio) {
    decision.reason = "LOW_INLIER_RATIO";
    return decision;
  }

  const Eigen::Isometry3d prediction_delta =
      predicted_map_T_base.inverse() * registration.map_T_base;
  decision.prediction_translation_m = planar_translation(prediction_delta.translation());
  decision.prediction_rotation_rad = yaw_angle(prediction_delta.rotation());
  if (decision.prediction_translation_m >
      config.max_prediction_translation_m) {
    decision.reason = "PREDICTION_TRANSLATION_JUMP";
    return decision;
  }
  if (decision.prediction_rotation_rad > config.max_prediction_rotation_rad) {
    decision.reason = "PREDICTION_ROTATION_JUMP";
    return decision;
  }

  decision.accepted = true;
  decision.reason = "OK";
  return decision;
}

Eigen::Isometry3d limit_map_T_odom_step(
    const Eigen::Isometry3d& current_map_T_odom,
    const Eigen::Isometry3d& candidate_map_T_odom,
    const TrackingConfig& config) {
  const Eigen::Isometry3d delta =
      current_map_T_odom.inverse() * candidate_map_T_odom;
  Eigen::Isometry3d limited_delta = Eigen::Isometry3d::Identity();

  const double translation_norm = delta.translation().norm();
  const double translation_scale =
      translation_norm > config.max_correction_translation_m &&
              translation_norm > 1e-12
          ? config.max_correction_translation_m / translation_norm
          : 1.0;
  limited_delta.translation() = delta.translation() * translation_scale;

  Eigen::Quaterniond delta_q(delta.rotation());
  delta_q.normalize();
  const double angle = rotation_angle(delta.rotation());
  const double rotation_scale =
      angle > config.max_correction_rotation_rad && angle > 1e-12
          ? config.max_correction_rotation_rad / angle
          : 1.0;
  limited_delta.linear() =
      Eigen::Quaterniond::Identity().slerp(rotation_scale, delta_q).toRotationMatrix();

  return current_map_T_odom * limited_delta;
}

const char* tracking_state_name(TrackingState state) {
  switch (state) {
    case TrackingState::WAITING_INITIALIZATION:
      return "WAITING_INITIALIZATION";
    case TrackingState::TRACKING:
      return "TRACKING";
    case TrackingState::DEGRADED:
      return "DEGRADED";
    case TrackingState::LOST:
      return "LOST";
  }
  return "UNKNOWN";
}

TrackingStateMachine::TrackingStateMachine(const TrackingConfig& config)
    : config_(config) {}

void TrackingStateMachine::initialize(double stamp_s) {
  initialized_ = true;
  state_ = TrackingState::TRACKING;
  last_accepted_stamp_s_ = stamp_s;
  consecutive_failures_ = 0;
  recovery_confirmations_ = 0;
}

TrackingState TrackingStateMachine::observe(bool accepted, double stamp_s) {
  if (!initialized_) {
    if (accepted) initialize(stamp_s);
    return state_;
  }

  if (accepted) {
    last_accepted_stamp_s_ = stamp_s;
    consecutive_failures_ = 0;
    if (state_ == TrackingState::TRACKING) {
      recovery_confirmations_ = 0;
      return state_;
    }
    ++recovery_confirmations_;
    if (recovery_confirmations_ >=
        std::max(1, config_.recovery_confirmations)) {
      state_ = TrackingState::TRACKING;
      recovery_confirmations_ = 0;
    }
    return state_;
  }

  recovery_confirmations_ = 0;
  ++consecutive_failures_;
  if (stamp_s - last_accepted_stamp_s_ >= config_.lost_after_s) {
    state_ = TrackingState::LOST;
  } else if (consecutive_failures_ >=
             std::max(1, config_.degraded_after_failures)) {
    state_ = TrackingState::DEGRADED;
  }
  return state_;
}

}  // namespace static_livox_localization

