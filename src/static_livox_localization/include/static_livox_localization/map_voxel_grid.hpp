#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <vector>

namespace static_livox_localization {

// A hash-map-backed occupancy grid built from the reference map at startup.
//
// At runtime, each incoming scan point (transformed to map frame via the
// predicted pose) is classified: if its voxel is empty in the map AND
// no occupied voxel exists within a search radius, the point is in
// "unmapped space" and kept; if its voxel is empty but occupied voxels
// exist nearby, the point is in "mapped empty space" — a pedestrian, a
// parked car that was not there when the map was built, or any other
// dynamic object that has no business pulling the registration.
//
// This is the root-cause fix for the pedestrian-induced pose jump
// (2026-08-09, 2026-08-11): GICP matches a pedestrian's returns to the
// nearest map structure and the fit looks excellent by every quality
// metric, but it is excellent about the wrong solution because a person
// is not in the map. Dynamic boxes (filter_dynamic_returns) require 1.5 s
// of tracking before they fire; this filter works on the first frame.
class MapVoxelGrid {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  // Build from a map cloud.  Voxel side is the same 0.20 m the rest of the
  // pipeline uses.  The map is voxelised at 2x the query resolution so a
  // single map point covers a slightly larger neighbourhood than a single
  // scan point — a scan point landing on the edge of a wall voxel should
  // not be called dynamic because the map's voxel is one step over.
  explicit MapVoxelGrid(const Cloud::ConstPtr& map,
                         double voxel_size_m = 0.20);

  // True if the point at (x, y, z) in map frame sits on map structure
  // within one voxel of measurement/pose uncertainty. Points farther away
  // but still inside search_radius_m of mapped structure occupy known empty
  // space and are removed. Points beyond the map's observable neighbourhood
  // remain UNKNOWN and are conservatively kept.
  bool is_likely_static(float x, float y, float z,
                        double search_radius_m = 0.40) const;

  // Filter a cloud in map frame: remove points classified as dynamic.
  // Returns the number of points removed. max_dropped_fraction remains for
  // configuration compatibility; confidently map-novel returns are never
  // restored because a crowd dominates the scan.
  std::size_t filter_dynamic(
      Cloud& cloud_in_map_frame,
      double search_radius_m = 0.40,
      double max_dropped_fraction = 0.50) const;

  std::size_t voxel_count() const { return occupied_.size(); }

 private:
  struct CellKey {
    int x, y, z;
    bool operator==(const CellKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
      // Good enough for a spatial hash; the domain is bounded by the map.
      return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 16) ^
             (std::hash<int>()(k.z) << 31);
    }
  };

  double voxel_size_m_;
  double inv_voxel_size_;
  std::unordered_map<CellKey, bool, CellKeyHash> occupied_;
  std::unordered_map<CellKey, std::vector<Eigen::Vector3f>, CellKeyHash>
      points_;

  CellKey key_for(double x, double y, double z) const;
  bool any_occupied_near(const CellKey& key, int half_range) const;
  bool any_point_within(float x, float y, float z, int half_range,
                        double radius_m) const;
};

}  // namespace static_livox_localization
