cmake_minimum_required(VERSION 2.8.3)
project(rbkairos_path_nav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/path_planner.py
  scripts/path_gui.py
  scripts/path_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  paths
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
