cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_safety)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf2_msgs tf2_ros wheelchair_interfaces)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/safety_gate.py
  scripts/mode_manager.py
  scripts/collision_supervisor.py
  scripts/slope_supervisor.py
  scripts/topology_guard.py
  scripts/localization_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
