cmake_minimum_required(VERSION 3.0.2)
project(static_livox_localization)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs diagnostic_msgs pcl_ros pcl_conversions)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenSSL REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES static_registration CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs diagnostic_msgs pcl_ros pcl_conversions)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})

add_library(static_registration src/registration.cpp)
target_link_libraries(static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_icp_localizer src/static_icp_localizer.cpp)
target_link_libraries(static_icp_localizer static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenSSL::Crypto)

catkin_install_python(PROGRAMS scripts/preflight.py scripts/report_static_trial.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS static_registration static_icp_localizer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_registration test/test_registration.cpp)
  if(TARGET test_registration)
    target_link_libraries(test_registration static_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()
