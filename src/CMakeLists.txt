# Workspace toplevel. catkin normally symlinks this to catkin's own
# toplevel.cmake, which declares `project(Project)` with the default C and CXX.
#
# Do not add `NONE` here. catkin snapshots the ENABLED_LANGUAGES global property
# while find_package(catkin) below is running, and defines catkin_add_gtest as a
# function that always fails when CXX is missing at that moment. The per-package
# project() calls enable C and CXX afterwards, which is too late: the workspace
# then fails to configure at the first catkin_add_gtest with
#   "catkin_find_google_test_source() is not available when the CXX language
#    isn't enabled."
cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_ros1_noetic_sim_ws)

find_package(catkin REQUIRED)
catkin_workspace()
