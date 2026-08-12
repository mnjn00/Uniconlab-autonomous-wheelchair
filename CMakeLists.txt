cmake_minimum_required(VERSION 3.16)
project(unicon_objective_regressions LANGUAGES CXX)

enable_testing()
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)

set(STATIC_LOCALIZATION
    "${CMAKE_CURRENT_SOURCE_DIR}/src/static_livox_localization")

add_executable(
  test_map_voxel_grid
  "${STATIC_LOCALIZATION}/src/map_voxel_grid.cpp"
  "${STATIC_LOCALIZATION}/test/test_map_voxel_grid.cpp")
target_compile_features(test_map_voxel_grid PRIVATE cxx_std_14)
target_include_directories(
  test_map_voxel_grid PRIVATE
  "${STATIC_LOCALIZATION}/include"
  ${PCL_INCLUDE_DIRS})
target_link_libraries(
  test_map_voxel_grid PRIVATE
  GTest::gtest
  GTest::gtest_main
  ${PCL_LIBRARIES})
add_test(NAME map_voxel_grid COMMAND test_map_voxel_grid)

add_executable(
  test_moving_tracker
  "${STATIC_LOCALIZATION}/src/moving_tracker.cpp"
  "${STATIC_LOCALIZATION}/src/registration.cpp"
  "${STATIC_LOCALIZATION}/test/test_moving_tracker.cpp")
target_compile_features(test_moving_tracker PRIVATE cxx_std_14)
target_include_directories(
  test_moving_tracker PRIVATE
  "${STATIC_LOCALIZATION}/include"
  ${PCL_INCLUDE_DIRS})
target_link_libraries(
  test_moving_tracker PRIVATE
  GTest::gtest
  GTest::gtest_main
  ${PCL_LIBRARIES})
add_test(NAME moving_tracker COMMAND test_moving_tracker)
