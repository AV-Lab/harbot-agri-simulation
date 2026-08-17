cmake_minimum_required(VERSION 3.0.2)
project(harbot_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_link_attacher
  harbot_control
  rospy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS gazebo_ros_link_attacher harbot_control rospy
)

catkin_install_python(PROGRAMS
  scripts/gazebo_attachment_adapter.py
  scripts/gt_odom_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch LICENSES models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
