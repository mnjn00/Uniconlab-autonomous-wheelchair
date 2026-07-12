cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_route_safety)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  rospy
  std_msgs
  wheelchair_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  rospy
  std_msgs
  wheelchair_interfaces
)

catkin_install_python(
  PROGRAMS scripts/route_safety.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  ../../data/hanyang_aegimun_loop/hanyang_aegimun_loop.waypoints.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES
  ../wheelchair_navigation/config/hanyang_routes.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES
  ../../contracts/wp0/A06-route-safety-schema.json
  ../../contracts/wp0/route-safety-candidate.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/contracts/wp0
)

install(PROGRAMS tests/test_route_safety.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
