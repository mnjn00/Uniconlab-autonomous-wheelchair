#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include "static_livox_localization/route_bounds.hpp"

namespace {

class TempFile {
 public:
  explicit TempFile(const std::string& contents) {
    std::vector<char> name(
        {'/', 't', 'm', 'p', '/', 'r', 'o', 'u', 't', 'e', '_', 'b', 'o',
         'u', 'n', 'd', 's', '_', 't', 'e', 's', 't', '_', 'X', 'X', 'X',
         'X', 'X', 'X', '\0'});
    const int fd = mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("mkstemp failed");
    close(fd);
    path_ = name.data();
    std::ofstream out(path_);
    out << contents;
  }
  ~TempFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

const char* kStationsJson = R"json(
{
 "frame": "map",
 "stations": [
  {"x": 0.28, "y": 0.15, "heading_deg": 12.6, "left_m": 0.3, "right_m": 0.9},
  {"x": 10.0, "y": 5.0, "heading_deg": 12.6, "left_m": 0.9, "right_m": 0.9}
 ]
}
)json";

}  // namespace

using static_livox_localization::load_route_bounds;
using static_livox_localization::RouteBounds;

TEST(RouteBounds, EmptyPathDisablesTheCheck) {
  const RouteBounds bounds = load_route_bounds("", 20.0);

  EXPECT_TRUE(bounds.empty());
  EXPECT_TRUE(bounds.contains(Eigen::Vector2d(1000.0, -1000.0)));
}

TEST(RouteBounds, LoadsStationsAndAcceptsPointsWithinMargin) {
  TempFile file(kStationsJson);
  const RouteBounds bounds = load_route_bounds(file.path(), 5.0);

  ASSERT_EQ(bounds.points.size(), 2u);
  EXPECT_TRUE(bounds.contains(Eigen::Vector2d(0.5, 0.2)));
  EXPECT_TRUE(bounds.contains(Eigen::Vector2d(12.0, 6.0)));
}

TEST(RouteBounds, RejectsTheRecurringWrongAttractorLocation) {
  // (171, -34) is the wrong-but-self-consistent ICP lock observed in two
  // separate field bags (2026-07-25 and 2026-07-26) on the real route,
  // whose stations sit near (0, 0)..(a few hundred metres away is not the
  // route). This is the exact failure mode this gate exists to catch.
  TempFile file(kStationsJson);
  const RouteBounds bounds = load_route_bounds(file.path(), 20.0);

  EXPECT_FALSE(bounds.contains(Eigen::Vector2d(171.23, -34.00)));
}

TEST(RouteBounds, MarginIsInclusiveAtTheBoundary) {
  TempFile file(kStationsJson);
  const RouteBounds bounds = load_route_bounds(file.path(), 5.0);

  // (5.28, 0.15) is exactly 5.0 m from the first station (0.28, 0.15).
  EXPECT_TRUE(bounds.contains(Eigen::Vector2d(5.28, 0.15)));
  EXPECT_FALSE(bounds.contains(Eigen::Vector2d(5.29, 0.15)));
}

TEST(RouteBounds, MissingFileThrows) {
  EXPECT_THROW(load_route_bounds("/nonexistent/path/route.json", 20.0),
               std::runtime_error);
}

TEST(RouteBounds, FileWithoutStationsThrows) {
  TempFile file("{\"frame\": \"map\", \"stations\": []}");

  EXPECT_THROW(load_route_bounds(file.path(), 20.0), std::runtime_error);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
