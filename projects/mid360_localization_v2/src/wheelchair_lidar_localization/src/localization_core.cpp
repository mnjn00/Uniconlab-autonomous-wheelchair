#include "wheelchair_lidar_localization/localization_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheelchair_lidar_localization {

bool registrationAccepted(const RegistrationMetrics& metrics,
                          const AcceptanceLimits& limits) {
  return metrics.converged &&
         metrics.source_points >= limits.min_source_points &&
         metrics.target_points >= limits.min_target_points &&
         std::isfinite(metrics.fitness) &&
         metrics.fitness >= 0.0 &&
         metrics.fitness <= limits.max_fitness &&
         std::isfinite(metrics.inlier_ratio) &&
         metrics.inlier_ratio >= limits.min_inlier_ratio &&
         metrics.inlier_ratio <= 1.0 &&
         std::isfinite(metrics.translation_correction_m) &&
         metrics.translation_correction_m >= 0.0 &&
         metrics.translation_correction_m <=
             limits.max_translation_correction_m &&
         std::isfinite(metrics.rotation_correction_rad) &&
         metrics.rotation_correction_rad >= 0.0 &&
         metrics.rotation_correction_rad <=
             limits.max_rotation_correction_rad;
}

double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawOf(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

double translationDistance(const Eigen::Isometry3d& lhs,
                           const Eigen::Isometry3d& rhs) {
  return (lhs.translation() - rhs.translation()).norm();
}

double rotationDistance(const Eigen::Isometry3d& lhs,
                        const Eigen::Isometry3d& rhs) {
  Eigen::Quaterniond delta(lhs.rotation().transpose() * rhs.rotation());
  delta.normalize();
  const double absolute_w =
      std::max(0.0, std::min(1.0, std::abs(delta.w())));
  return 2.0 * std::acos(absolute_w);
}

double tiltAngle(const Eigen::Isometry3d& pose) {
  const double aligned_z =
      std::max(-1.0, std::min(1.0, pose.rotation()(2, 2)));
  return std::acos(aligned_z);
}

PoseConsensus::PoseConsensus(std::size_t required,
                             double translation_tolerance_m,
                             double rotation_tolerance_rad)
    : required_(required),
      translation_tolerance_m_(translation_tolerance_m),
      rotation_tolerance_rad_(rotation_tolerance_rad) {
  if (required_ == 0 || translation_tolerance_m_ <= 0.0 ||
      rotation_tolerance_rad_ <= 0.0) {
    throw std::invalid_argument("invalid pose consensus configuration");
  }
}

bool PoseConsensus::add(const Eigen::Isometry3d& map_T_odom) {
  if (!map_T_odom.matrix().allFinite()) {
    clear();
    return false;
  }
  if (!samples_.empty()) {
    const Eigen::Isometry3d reference = mean();
    if (translationDistance(reference, map_T_odom) >
            translation_tolerance_m_ ||
        rotationDistance(reference, map_T_odom) >
            rotation_tolerance_rad_) {
      clear();
    }
  }
  samples_.push_back(map_T_odom);
  while (samples_.size() > required_) {
    samples_.pop_front();
  }
  return samples_.size() == required_;
}

void PoseConsensus::clear() { samples_.clear(); }

std::size_t PoseConsensus::size() const { return samples_.size(); }

Eigen::Isometry3d PoseConsensus::mean() const {
  if (samples_.empty()) {
    throw std::runtime_error("pose consensus is empty");
  }
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion_coefficients = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond first(samples_.front().rotation());
  for (const auto& sample : samples_) {
    translation += sample.translation();
    Eigen::Quaterniond q(sample.rotation());
    if (q.coeffs().dot(first.coeffs()) < 0.0) {
      q.coeffs() *= -1.0;
    }
    quaternion_coefficients += q.coeffs();
  }
  translation /= static_cast<double>(samples_.size());
  Eigen::Quaterniond rotation(
      quaternion_coefficients.w(), quaternion_coefficients.x(),
      quaternion_coefficients.y(), quaternion_coefficients.z());
  rotation.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = translation;
  result.linear() = rotation.toRotationMatrix();
  return result;
}

}  // namespace wheelchair_lidar_localization
