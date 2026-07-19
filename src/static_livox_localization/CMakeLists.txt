cmake_minimum_required(VERSION 3.0.2)
project(static_livox_localization)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs diagnostic_msgs nav_msgs tf2 tf2_ros tf2_msgs visualization_msgs message_filters pcl_ros pcl_conversions)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenSSL REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES static_registration moving_tracker rolling_submap CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs diagnostic_msgs nav_msgs tf2 tf2_ros tf2_msgs visualization_msgs message_filters pcl_ros pcl_conversions)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})

add_library(static_registration src/registration.cpp)
target_link_libraries(static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(moving_tracker src/moving_tracker.cpp)
target_link_libraries(moving_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(rolling_submap src/rolling_submap.cpp)
target_link_libraries(rolling_submap ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(static_icp_localizer src/static_icp_localizer.cpp)
target_link_libraries(static_icp_localizer static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_executable(moving_icp_localizer src/moving_icp_localizer.cpp)
target_link_libraries(moving_icp_localizer rolling_submap moving_tracker static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)
add_executable(map_preview_publisher src/map_preview_publisher.cpp)
target_link_libraries(map_preview_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)



catkin_install_python(PROGRAMS scripts/preflight.py scripts/report_static_trial.py scripts/report_moving_trial.py scripts/replay_metrics.py scripts/reference_marker.py scripts/localization_state_marker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS static_registration moving_tracker rolling_submap static_icp_localizer moving_icp_localizer map_preview_publisher RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
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
endif()
