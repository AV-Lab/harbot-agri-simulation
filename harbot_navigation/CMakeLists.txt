cmake_minimum_required(VERSION 2.8.3)
project(harbot_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/navigation_launcher.py
  scripts/path_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  paths
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  scripts/global_planners
  scripts/local_planners
  scripts/speed_planners
  scripts/controllers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(FILES
  scripts/recorded_route.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
