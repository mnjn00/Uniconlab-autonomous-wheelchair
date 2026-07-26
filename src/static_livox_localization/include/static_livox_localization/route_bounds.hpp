#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace static_livox_localization {

// A geometric plausibility gate for relocalization candidates: independent
// of ICP fitness/inlier metrics, which only measure how well a candidate
// pose's local scan matches nearby map points and say nothing about whether
// that neighborhood is the RIGHT place. A distant region of the map that
// happens to look structurally similar to the true route can still pass
// fitness/inlier/consensus checks (aliasing), converge with plenty of
// target points, and get accepted as ground truth. This gate rejects any
// candidate whose position is farther than `margin_m` from every known
// route/safety-band station, regardless of how good its registration score
// looks.
struct RouteBounds {
  std::vector<Eigen::Vector2d> points;
  double margin_m = 20.0;

  // No points loaded (route_bounds_path left empty) means the check is
  // disabled - existing deployments without the parameter set keep their
  // current behavior.
  bool empty() const { return points.empty(); }

  bool contains(const Eigen::Vector2d& xy) const;
};

// Route/safety-band files in this project are flat station lists shaped
// like {"x": <num>, ..., "y": <num>, ...} (see routes/*_safety_band.json).
// This pulls every "x"/"y" pair in file order rather than parsing general
// JSON, to avoid adding a JSON library dependency for one bounding check.
// Throws if the path is non-empty but cannot be opened or yields no usable
// x/y pairs, so a misconfigured path fails closed at startup rather than
// silently disabling the gate.
RouteBounds load_route_bounds(const std::string& path, double margin_m);

}  // namespace static_livox_localization
