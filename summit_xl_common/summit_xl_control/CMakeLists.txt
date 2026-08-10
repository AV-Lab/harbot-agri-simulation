cmake_minimum_required(VERSION 3.0.2)
project(summit_xl_control)


find_package(catkin REQUIRED
    diff_drive_controller
    joint_state_controller
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
