cmake_minimum_required(VERSION 3.0.2)
project(harbot_control)

find_package(catkin REQUIRED COMPONENTS
  diff_drive_controller
  joint_state_controller
)

catkin_package()

install(
  DIRECTORY config launch LICENSES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
