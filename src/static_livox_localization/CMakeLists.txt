cmake_minimum_required(VERSION 3.0.2)
project(static_livox_localization)

option(ENABLE_FAST_GICP_CUDA "Enable the fail-closed fast_vgicp CUDA registration backend" OFF)

set(STATIC_LIVOX_OPTIONAL_CATKIN_COMPONENTS)
if(ENABLE_FAST_GICP_CUDA)
  list(APPEND STATIC_LIVOX_OPTIONAL_CATKIN_COMPONENTS fast_gicp)
endif()

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs diagnostic_msgs nav_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs tf2_msgs visualization_msgs message_filters pcl_ros pcl_conversions ${STATIC_LIVOX_OPTIONAL_CATKIN_COMPONENTS})
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenSSL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES static_registration moving_tracker rolling_submap assisted_alignment bounded_cloud_preview
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs diagnostic_msgs nav_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs tf2_msgs visualization_msgs message_filters pcl_ros pcl_conversions
)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})

add_library(static_registration src/registration.cpp)
target_link_libraries(static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
if(ENABLE_FAST_GICP_CUDA)
  target_compile_definitions(static_registration PRIVATE STATIC_LIVOX_HAS_FAST_VGICP_CUDA=1)
endif()
add_library(moving_tracker src/moving_tracker.cpp)
target_link_libraries(moving_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(rolling_submap src/rolling_submap.cpp)
target_link_libraries(rolling_submap ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(map_voxel_grid src/map_voxel_grid.cpp)
target_link_libraries(map_voxel_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(assisted_alignment src/assisted_alignment.cpp)
target_link_libraries(assisted_alignment ${catkin_LIBRARIES})
add_library(bounded_cloud_preview src/bounded_cloud_preview.cpp)
target_link_libraries(bounded_cloud_preview ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_icp_localizer src/static_icp_localizer.cpp)
target_link_libraries(static_icp_localizer static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_executable(moving_icp_localizer src/moving_icp_localizer.cpp)
target_link_libraries(moving_icp_localizer assisted_alignment rolling_submap moving_tracker static_registration map_voxel_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_executable(map_preview_publisher src/map_preview_publisher.cpp)
target_link_libraries(map_preview_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_executable(bounded_cloud_preview_node src/bounded_cloud_preview_node.cpp)
target_link_libraries(bounded_cloud_preview_node bounded_cloud_preview ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(
  PROGRAMS
    scripts/preflight.py
    scripts/stop_watchdog.py
    scripts/report_static_trial.py
    scripts/report_moving_trial.py
    scripts/replay_metrics.py
    scripts/route_mask.py
    scripts/route_identity_publisher.py
    scripts/reference_marker.py
    scripts/localization_state_marker.py
    scripts/auto_initial_pose.py
    scripts/fastlio_init_health.py
    scripts/enable_auto_correction_after_seed.py
    scripts/tip_guard.py
    scripts/waypoint_follower.py
    scripts/mpc_follower.py
    scripts/dwa_follower.py
    scripts/safety_gate.py
    scripts/obstacle_clusters.py
    scripts/hybrid_object_fusion.py
    scripts/vision_detection_bridge.py
    scripts/semantic_safety_supervisor.py
    scripts/terrain_guard.py
    scripts/hybrid_preflight.py
    scripts/dwa_run_report.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  FILES
    scripts/array_backend.py
    scripts/gpu_voxel_scorer.py
    scripts/body_frame.py
    scripts/dwa_core.py
    scripts/hybrid_perception.py
    scripts/semantic_safety_policy.py
    scripts/terrain_guard_policy.py
    scripts/initial_pose_candidates.py
    scripts/initial_pose_global_search.py
    scripts/cluster_guard.py
    scripts/cluster_tracking.py
    scripts/cloud_points.py
    scripts/fixed_map_filter.py
    scripts/drive_policy.py
    scripts/localization_policy.py
    scripts/motion_safety.py
    scripts/mpc_anchor.py
    scripts/mpc_command.py
    scripts/mpc_core.py
    scripts/mpc_speed.py
    scripts/safety_band.py
    scripts/route_assets.py
    scripts/scan_accumulator.py
    scripts/shadow_qa_contract.py
    scripts/tip_guard_policy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS
    static_registration
    moving_tracker
    rolling_submap
    assisted_alignment
    bounded_cloud_preview
    static_icp_localizer
    moving_icp_localizer
    map_preview_publisher
    bounded_cloud_preview_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_registration test/test_registration.cpp)
  if(TARGET test_registration)
    target_link_libraries(test_registration static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()

  catkin_add_gtest(test_moving_tracker test/test_moving_tracker.cpp)
  if(TARGET test_moving_tracker)
    target_link_libraries(test_moving_tracker moving_tracker static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()

  catkin_add_gtest(test_rolling_submap test/test_rolling_submap.cpp)
  if(TARGET test_rolling_submap)
    target_link_libraries(test_rolling_submap rolling_submap ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(test_map_voxel_grid test/test_map_voxel_grid.cpp)
  if(TARGET test_map_voxel_grid)
    target_link_libraries(test_map_voxel_grid map_voxel_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()

  catkin_add_gtest(test_assisted_alignment test/test_assisted_alignment.cpp)
  if(TARGET test_assisted_alignment)
    target_link_libraries(test_assisted_alignment assisted_alignment ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_bounded_cloud_preview test/test_bounded_cloud_preview.cpp)
  if(TARGET test_bounded_cloud_preview)
    target_link_libraries(test_bounded_cloud_preview bounded_cloud_preview ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()
