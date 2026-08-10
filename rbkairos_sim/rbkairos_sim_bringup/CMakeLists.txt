cmake_minimum_required(VERSION 3.0.2)
project(rbkairos_sim_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/approach_orange.py
  scripts/container_control.py
  scripts/fake_orange_pose_pub.py
  scripts/grasp_orange.py
  scripts/gt_odom_tf.py
  scripts/harvest_workflow.py
  scripts/hsv_object_detector.py
  scripts/move_franka_to_joint_pose.py
  scripts/moveit_target_reach.py
  scripts/orange_pose_to_world.py
  scripts/print_franka_joint_values.py
  scripts/unload_orange.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
