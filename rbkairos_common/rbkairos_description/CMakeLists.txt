cmake_minimum_required(VERSION 3.0.2)
project(rbkairos_description)

find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY meshes urdf robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
