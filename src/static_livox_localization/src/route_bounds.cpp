#include "static_livox_localization/route_bounds.hpp"

#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace static_livox_localization {

bool RouteBounds::contains(const Eigen::Vector2d& xy) const {
  if (points.empty()) return true;
  double best = std::numeric_limits<double>::infinity();
  for (const auto& p : points) {
    const double d = (xy - p).norm();
    if (d < best) best = d;
  }
  return best <= margin_m;
}

RouteBounds load_route_bounds(const std::string& path, double margin_m) {
  RouteBounds bounds;
  bounds.margin_m = margin_m;
  if (path.empty()) return bounds;

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open route bounds file: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string text = buffer.str();

  static const std::regex x_re(
      R"("x"\s*:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?))");
  static const std::regex y_re(
      R"("y"\s*:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?))");

  std::vector<double> xs;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), x_re);
       it != std::sregex_iterator(); ++it) {
    xs.push_back(std::stod((*it)[1].str()));
  }
  std::vector<double> ys;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), y_re);
       it != std::sregex_iterator(); ++it) {
    ys.push_back(std::stod((*it)[1].str()));
  }

  if (xs.empty() || xs.size() != ys.size()) {
    throw std::runtime_error(
        "route bounds file has no usable x/y station pairs: " + path);
  }
  bounds.points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    bounds.points.emplace_back(xs[i], ys[i]);
  }
  return bounds;
}

}  // namespace static_livox_localization
