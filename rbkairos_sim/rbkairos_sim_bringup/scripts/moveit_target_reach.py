#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

# Object properties
OBJECT_DIAMETER = 0.07
OFFSET_X = -0.0005
OFFSET_Y = 0.00116
OFFSET_Z = -0.09296

# Object position in world frame 
OBJECT_X = -1.00881 + OFFSET_X
OBJECT_Y = 1.4173 + OFFSET_Y
OBJECT_Z = 1.44663 + OFFSET_Z

# Arm target position (can differ from the visualized object position)
TARGET_X = OBJECT_X
TARGET_Y = OBJECT_Y
TARGET_Z = OBJECT_Z

# Collision avoidance box (above the object)
BOX_X = -1.01061 + OFFSET_X
BOX_Y = 1.4173 + OFFSET_Y
BOX_Z = 1.51907 + OFFSET_Z
BOX_SIZE = (0.06, 0.06, 0.08)

# ROS configuration
MARKER_FRAME = "robot_map"
ROBOT_NS = "/robot"

# For Franka, use:
MOVE_GROUP_NAME = "arm"
EE_LINK = 'robot_hand_tcp'

# Planning parameters
PLANNER_ID = "OMPL"
PLANNING_TIME = 10.0
PLANNING_ATTEMPTS = 10
POSITION_TOLERANCE = 0.01


def create_object_marker():
    marker = Marker()
    marker.header.frame_id = MARKER_FRAME
    marker.ns = "detected_object"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position.x = OBJECT_X
    marker.pose.position.y = OBJECT_Y
    marker.pose.position.z = OBJECT_Z
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = OBJECT_DIAMETER
    marker.scale.y = OBJECT_DIAMETER
    marker.scale.z = OBJECT_DIAMETER
    
    marker.color.r = 1.0
    marker.color.g = 0.45
    marker.color.b = 0.0
    marker.color.a = 0.9
    
    return marker


def create_target_marker():
    marker = Marker()
    marker.header.frame_id = MARKER_FRAME
    marker.ns = "target_point"
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position.x = TARGET_X
    marker.pose.position.y = TARGET_Y
    marker.pose.position.z = TARGET_Z
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    
    return marker


def add_collision_objects(scene):
    # Add sphere for the object
    pose = PoseStamped()
    pose.header.frame_id = MARKER_FRAME
    pose.pose.position.x = OBJECT_X
    pose.pose.position.y = OBJECT_Y
    pose.pose.position.z = OBJECT_Z
    pose.pose.orientation.w = 1.0
    
    scene.add_sphere("detected_object", pose, radius=OBJECT_DIAMETER / 2.0)
    rospy.loginfo("Added collision sphere at [%.3f, %.3f, %.3f]", OBJECT_X, OBJECT_Y, OBJECT_Z)
    
    # Add box above the object for collision avoidance
    pose_box = PoseStamped()
    pose_box.header.frame_id = MARKER_FRAME
    pose_box.pose.position.x = BOX_X
    pose_box.pose.position.y = BOX_Y
    pose_box.pose.position.z = BOX_Z
    pose_box.pose.orientation.w = 1.0
    
    scene.add_box("object_above", pose_box, size=BOX_SIZE)
    rospy.loginfo("Added collision box at [%.3f, %.3f, %.3f]", BOX_X, BOX_Y, BOX_Z)


def move_arm_to_target():
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    scene = PlanningSceneInterface(ns=ROBOT_NS)
    group = moveit_commander.MoveGroupCommander(
        MOVE_GROUP_NAME,
        robot_description=f"{ROBOT_NS}/robot_description",
        ns=ROBOT_NS
    )
    
    # Configure planner
    group.set_planner_id(PLANNER_ID)
    group.set_planning_time(PLANNING_TIME)
    group.set_num_planning_attempts(PLANNING_ATTEMPTS)
    group.set_goal_position_tolerance(POSITION_TOLERANCE)
    
    # Determine end-effector link
    ee_link = EE_LINK if EE_LINK else group.get_end_effector_link()
    rospy.loginfo(
        "Using end-effector: %s | Target: [%.5f, %.5f, %.5f]",
        ee_link, TARGET_X, TARGET_Y, TARGET_Z
    )
    
    # Set up collision scene
    rospy.sleep(1.0)  
    add_collision_objects(scene)
    rospy.sleep(5.0)  # Wait for scene to update
    
    # Plan and move
    group.set_pose_reference_frame(MARKER_FRAME)
    group.set_position_target([TARGET_X, TARGET_Y, TARGET_Z], ee_link)
    
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    rospy.loginfo("Arm movement %s", "succeeded" if success else "FAILED")
    moveit_commander.roscpp_shutdown()
    
    return success


def main():
    rospy.init_node("moveit_target_reach")
    
    arm_type = rospy.get_param("~arm", "Franka")
    if arm_type.lower() == "ur5":
        global MOVE_GROUP_NAME, EE_LINK
        MOVE_GROUP_NAME = "ur5_arm"
        EE_LINK = "robot_egh_gripper_grasp_link"
        rospy.loginfo("Configured for UR5 arm")
    else:
        rospy.loginfo("Configured for Franka arm")
    
    # Create publishers
    object_marker_pub = rospy.Publisher("/detected_object_marker", Marker, queue_size=1)
    target_marker_pub = rospy.Publisher("/target_point_marker", Marker, queue_size=1)
    
    # Create markers
    object_marker = create_object_marker()
    target_marker = create_target_marker()
    
    # Execute movement once
    moved = False
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        # Update and publish markers
        object_marker.header.stamp = rospy.Time.now()
        target_marker.header.stamp = rospy.Time.now()
        object_marker_pub.publish(object_marker)
        target_marker_pub.publish(target_marker)
        
        # Execute movement on first loop iteration
        if not moved:
            moved = True
            move_arm_to_target()
        
        rate.sleep()


if __name__ == "__main__":
    main()