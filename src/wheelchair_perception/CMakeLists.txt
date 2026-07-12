cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_perception)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs diagnostic_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/perception_core.py
  scripts/perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
