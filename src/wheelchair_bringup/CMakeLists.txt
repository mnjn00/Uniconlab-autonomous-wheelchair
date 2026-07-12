cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_bringup)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  wheelchair_interfaces
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs rospy sensor_msgs wheelchair_interfaces
)

catkin_install_python(PROGRAMS
  scripts/incident_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
