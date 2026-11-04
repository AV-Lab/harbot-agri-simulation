cmake_minimum_required(VERSION 3.0.2)
project(harbot_harvest)

find_package(catkin REQUIRED COMPONENTS
  harbot_control
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS harbot_control
)

catkin_install_python(PROGRAMS
  scripts/approach_orange.py
  scripts/container_control.py
  scripts/grasp_orange.py
  scripts/harvest_workflow.py
  scripts/hsv_object_detector.py
  scripts/moveit_target_reach.py
  scripts/prost_sim_action_server.py
  scripts/unload_orange.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
