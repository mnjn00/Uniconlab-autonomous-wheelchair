cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_msgs
  tf2_ros
  wheelchair_interfaces
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_msgs
  tf2_ros
  wheelchair_interfaces
)
catkin_install_python(PROGRAMS
  scripts/localization_adapter.py
  scripts/route_manager.py
  scripts/control_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
