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
  // Static points on the wall — in a real scan the wall produces
  // far more returns than a pedestrian, so we generate a dense wall.
  for (float x = 0.5f; x < 4.5f; x += 0.05f) {
    for (float z = 0.0f; z < 2.0f; z += 0.1f) {
      pcl::PointXYZI p;
      p.x = x;
      p.y = 0.0f;
      p.z = z;
      p.intensity = 1.0f;
      scan->push_back(p);
    }
  }
  // Person at y=0.3 — in mapped empty space, within 0.40 m of the
  // wall at y=0 but not on the wall itself.
  for (float py = 0.25f; py < 0.45f; py += 0.05f) {
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
  // Point at y=0.3 — within 0.40 of the wall at y=0, in mapped empty space
  EXPECT_FALSE(grid.is_likely_static(2.0f, 0.3f, 0.5f, 0.40));
  // Point at y=2.0 — far from wall, in unmapped space — kept
  EXPECT_TRUE(grid.is_likely_static(2.0f, 2.0f, 0.5f, 0.40));
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
  for (const auto& p : scan->points) {
    EXPECT_FLOAT_EQ(p.intensity, 1.0f);
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

TEST(MapVoxelGrid, MappedStaticReturnAcrossVoxelBoundaryIsKept) {
  Cloud::Ptr map(new Cloud);
  pcl::PointXYZI mapped;
  mapped.x = 0.19f;
  mapped.y = mapped.z = 0.0f;
  map->push_back(mapped);
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  EXPECT_TRUE(grid.is_likely_static(0.21f, 0.0f, 0.0f, 0.40));
}

TEST(MapVoxelGrid, CrowdDominatedScanStillKeepsMapAndDropsPeople) {
  Cloud::Ptr map(new Cloud);
  pcl::PointXYZI wall;
  wall.x = wall.y = wall.z = 0.0f;
  wall.intensity = 1.0f;
  map->push_back(wall);
  Cloud scan;
  for (int i = 0; i < 40; ++i) {
    scan.push_back(wall);
  }
  for (int i = 0; i < 60; ++i) {
    pcl::PointXYZI person;
    person.x = 0.0f;
    person.y = 0.21f;
    person.z = 0.0f;
    person.intensity = 2.0f;
    scan.push_back(person);
  }
  static_livox_localization::MapVoxelGrid grid(map, 0.20);
  EXPECT_EQ(grid.filter_dynamic(scan, 0.40, 0.50), 60u);
  ASSERT_EQ(scan.size(), 40u);
  for (const auto& point : scan.points) {
    EXPECT_FLOAT_EQ(point.intensity, 1.0f);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
