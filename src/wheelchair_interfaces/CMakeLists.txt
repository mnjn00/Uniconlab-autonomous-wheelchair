cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_interfaces)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  SafetyReason.msg
  SafetySignal.msg
  CollisionStatus.msg
  SlopeStatus.msg
  LocalizationCandidate.msg
  LocalizationStatus.msg
  ActiveRoute.msg
  GeofenceStatus.msg
  MotionIntent.msg
  RouteProgress.msg
  MissionState.msg
  DriverStatus.msg
  SafetyState.msg
)

add_action_files(
  FILES
  ExecuteRoute.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  std_msgs
)

install(DIRECTORY msg action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
