cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_hardware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  nav_msgs
  rospy
  std_msgs
  wheelchair_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  nav_msgs
  rospy
  std_msgs
  wheelchair_interfaces
)

catkin_install_python(
  PROGRAMS
  scripts/driver_contract.py
  scripts/hardware_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS tests/test_driver_contract.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
