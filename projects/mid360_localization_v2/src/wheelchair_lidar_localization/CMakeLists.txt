cmake_minimum_required(VERSION 3.10)
project(wheelchair_lidar_localization)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  fast_gicp
  geometry_msgs
  hdl_global_localization
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wheelchair_localization_core
  CATKIN_DEPENDS diagnostic_msgs fast_gicp geometry_msgs hdl_global_localization
                 nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs
                 tf2 tf2_eigen tf2_ros
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(wheelchair_localization_core src/localization_core.cpp)
target_link_libraries(wheelchair_localization_core ${catkin_LIBRARIES})

add_executable(wheelchair_map_localizer src/map_localizer_node.cpp)
target_link_libraries(
  wheelchair_map_localizer
  wheelchair_localization_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenSSL::Crypto
)

catkin_install_python(
  PROGRAMS scripts/localization_preflight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS wheelchair_localization_core wheelchair_map_localizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_localization_core test/test_localization_core.cpp)
  if(TARGET test_localization_core)
    target_link_libraries(test_localization_core wheelchair_localization_core ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_preflight.py)
  catkin_add_nosetests(test/test_surface.py)
endif()
