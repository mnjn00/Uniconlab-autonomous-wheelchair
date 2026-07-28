#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Geometry>

namespace wheelchair_lidar_localization {

enum class RawState : unsigned char {
  UNINITIALIZED = 0,
  INITIALIZING = 1,
  OK = 2,
  DEGRADED = 3,
  LOST = 4,
};

struct RegistrationMetrics {
  bool converged = false;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  double fitness = 1e9;
  double inlier_ratio = 0.0;
  double translation_correction_m = 1e9;
  double rotation_correction_rad = 1e9;
};

struct AcceptanceLimits {
  std::size_t min_source_points = 500;
  std::size_t min_target_points = 1000;
  double max_fitness = 0.20;
  double min_inlier_ratio = 0.35;
  double max_translation_correction_m = 0.35;
  double max_rotation_correction_rad = 0.10;
};

bool registrationAccepted(const RegistrationMetrics& metrics,
                          const AcceptanceLimits& limits);
double wrapAngle(double angle);
double yawOf(const Eigen::Isometry3d& pose);
double translationDistance(const Eigen::Isometry3d& lhs,
                           const Eigen::Isometry3d& rhs);
double rotationDistance(const Eigen::Isometry3d& lhs,
                        const Eigen::Isometry3d& rhs);
double tiltAngle(const Eigen::Isometry3d& pose);

class PoseConsensus {
 public:
  PoseConsensus(std::size_t required, double translation_tolerance_m,
                double rotation_tolerance_rad);

  bool add(const Eigen::Isometry3d& map_T_odom);
  void clear();
  std::size_t size() const;
  Eigen::Isometry3d mean() const;

 private:
  std::size_t required_;
  double translation_tolerance_m_;
  double rotation_tolerance_rad_;
  std::deque<Eigen::Isometry3d> samples_;
};

}  // namespace wheelchair_lidar_localization
