#include "static_livox_localization/map_voxel_grid.hpp"

#include <cmath>

namespace static_livox_localization {

MapVoxelGrid::MapVoxelGrid(const Cloud::ConstPtr& map, double voxel_size_m)
    : voxel_size_m_(voxel_size_m),
      inv_voxel_size_(1.0 / voxel_size_m) {
  if (!map || map->empty()) return;
  for (const auto& p : map->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    CellKey k = key_for(p.x, p.y, p.z);
    occupied_[k] = true;
  }
}

MapVoxelGrid::CellKey MapVoxelGrid::key_for(double x, double y, double z) const {
  CellKey k;
  k.x = static_cast<int>(std::floor(x * inv_voxel_size_));
  k.y = static_cast<int>(std::floor(y * inv_voxel_size_));
  k.z = static_cast<int>(std::floor(z * inv_voxel_size_));
  return k;
}

bool MapVoxelGrid::any_occupied_near(const CellKey& center, int half_range) const {
  for (int dx = -half_range; dx <= half_range; ++dx) {
    for (int dy = -half_range; dy <= half_range; ++dy) {
      for (int dz = -half_range; dz <= half_range; ++dz) {
        CellKey n{center.x + dx, center.y + dy, center.z + dz};
        if (occupied_.find(n) != occupied_.end()) return true;
      }
    }
  }
  return false;
}

bool MapVoxelGrid::is_likely_static(float x, float y, float z,
                                    double search_radius_m) const {
  CellKey k = key_for(x, y, z);
  // Point lands directly on map structure — keep it.
  if (occupied_.find(k) != occupied_.end()) return true;
  int half = static_cast<int>(std::ceil(search_radius_m * inv_voxel_size_));
  if (half < 1) half = 1;
  // Point is in an empty voxel but map structure exists nearby — this is
  // "mapped empty space". The map says nothing should be here, so the
  // return is from a pedestrian, a parked car that was not there when the
  // map was built, or any other dynamic object. Remove it.
  if (any_occupied_near(k, half)) return false;
  // Point is far from any map structure — "unmapped space". We cannot tell
  // whether it is a new static feature or a dynamic object, so we keep it
  // to avoid deleting structure the registration depends on.
  return true;
}

std::size_t MapVoxelGrid::filter_dynamic(
    Cloud& cloud_in_map_frame,
    double search_radius_m,
    double max_dropped_fraction) const {
  if (cloud_in_map_frame.empty() || occupied_.empty()) return 0;

  std::vector<char> drop(cloud_in_map_frame.size(), 0);
  std::size_t dropped = 0;
  for (std::size_t i = 0; i < cloud_in_map_frame.size(); ++i) {
    const auto& p = cloud_in_map_frame.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (!is_likely_static(p.x, p.y, p.z, search_radius_m)) {
      drop[i] = 1;
      ++dropped;
    }
  }
  if (dropped == 0) return 0;

  const double fraction =
      static_cast<double>(dropped) /
      static_cast<double>(cloud_in_map_frame.size());
  if (fraction > max_dropped_fraction) return 0;

  Cloud kept;
  kept.reserve(cloud_in_map_frame.size() - dropped);
  for (std::size_t i = 0; i < cloud_in_map_frame.size(); ++i) {
    if (!drop[i]) kept.push_back(cloud_in_map_frame.points[i]);
  }
  kept.header = cloud_in_map_frame.header;
  kept.is_dense = cloud_in_map_frame.is_dense;
  cloud_in_map_frame.swap(kept);
  return dropped;
}

}  // namespace static_livox_localization
