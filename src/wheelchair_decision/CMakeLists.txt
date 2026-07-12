cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_decision)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  move_base_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
  wheelchair_interfaces
  wheelchair_navigation
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  move_base_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
  wheelchair_interfaces
  wheelchair_navigation
)

catkin_install_python(
  PROGRAMS
  scripts/mission_core.py
  scripts/mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS tests/test_mission_core.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
