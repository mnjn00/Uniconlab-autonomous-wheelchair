cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_safety)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs std_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/safety_gate.py
  scripts/mode_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
