#include <cmath>

#include <gtest/gtest.h>

#include "static_livox_localization/map_voxel_grid.hpp"

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Cloud::Ptr make_map_with_wall() {
  Cloud::Ptr map(new Cloud);
  for (float x = 0.0f; x < 5.0f; x += 0.1f) {
    for (float z = 0.0f; z < 2.0f; z += 0.1f) {
      pcl::PointXYZI p;
      p.x = x;
      p.y = 0.0f;
      p.z = z;
      p.intensity = 1.0f;
      map->push_back(p);
    }
  }
  return map;
}

Cloud::Ptr make_scan_with_person() {
  Cloud::Ptr scan(new Cloud);
  // Static points on the wall
  for (float x = 1.0f; x < 3.0f; x += 0.1f) {
    pcl::PointXYZI p;
    p.x = x;
    p.y = 0.0f;
    p.z = 0.5f;
    p.intensity = 1.0f;
    scan->push_back(p);
  }
  // Person at y=2.0 — not in the map
  for (float py = 1.8f; py < 2.2f; py += 0.05f) {
    for (float pz = 0.2f; pz < 1.8f; pz += 0.1f) {
      pcl::PointXYZI p;
      p.x = 2.0f;
      p.y = py;
      p.z = pz;
      p.intensity = 1.0f;
      scan->push_back(p);
    }
  }
  return scan;
}

}  // namespace

TEST(MapVoxelGrid, PointInMapIsStatic) {
  auto map = make_map_with_wall();
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  EXPECT_TRUE(grid.is_likely_static(1.0f, 0.0f, 0.5f));
  EXPECT_TRUE(grid.is_likely_static(2.5f, 0.0f, 1.0f));
}

TEST(MapVoxelGrid, PointInMappedEmptySpaceIsDynamic) {
  auto map = make_map_with_wall();
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  // Point at y=2.0 — the wall is at y=0, so 2.0 m away
  // With search_radius 0.40, no occupied voxel is near -> unmapped, not dynamic
  EXPECT_FALSE(grid.is_likely_static(2.0f, 2.0f, 0.5f, 0.40));
  // But at y=0.3 — within 0.40 of the wall — should be static
  EXPECT_TRUE(grid.is_likely_static(2.0f, 0.3f, 0.5f, 0.40));
}

TEST(MapVoxelGrid, PointInUnmappedSpaceIsKept) {
  auto map = make_map_with_wall();
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  // Point far from any map structure — unmapped, keep it
  EXPECT_TRUE(grid.is_likely_static(10.0f, 10.0f, 0.5f, 0.40));
}

TEST(MapVoxelGrid, FilterRemovesPersonKeepsWall) {
  auto map = make_map_with_wall();
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  auto scan = make_scan_with_person();
  std::size_t before = scan->size();
  std::size_t dropped = grid.filter_dynamic(*scan, 0.40, 0.50);
  EXPECT_GT(dropped, 0u);
  EXPECT_LT(scan->size(), before);
  // Remaining points should all be near the wall (y close to 0)
  for (const auto& p : scan->points) {
    EXPECT_NEAR(p.y, 0.0f, 0.5f);
  }
}

TEST(MapVoxelGrid, EmptyMapIsNoop) {
  Cloud::Ptr empty(new Cloud);
  static_livox_localization::MapVoxelGrid grid(empty, 0.20);
  Cloud scan;
  pcl::PointXYZI p;
  p.x = p.y = p.z = 1.0f;
  scan.push_back(p);
  std::size_t dropped = grid.filter_dynamic(scan, 0.40, 0.50);
  EXPECT_EQ(dropped, 0u);
  EXPECT_EQ(scan.size(), 1u);
}

TEST(MapVoxelGrid, MaxDroppedFractionPreventsOverFiltering) {
  auto map = make_map_with_wall();
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  auto scan = make_scan_with_person();
  // Set max_dropped_fraction to 0.01 — almost nothing can be dropped
  std::size_t dropped = grid.filter_dynamic(*scan, 0.40, 0.01);
  EXPECT_EQ(dropped, 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
