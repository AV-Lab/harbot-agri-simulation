#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Robot and object config
ROBOT_MODEL = "robot"
ORANGE_MODEL = "orange2.5.R"
ORANGE_LINK = "body"

# Arm config
ARM_JOINTS = [
    "robot_arm_shoulder_pan_joint",
    "robot_arm_shoulder_lift_joint",
    "robot_arm_elbow_joint",
    "robot_arm_wrist_1_joint",
    "robot_arm_wrist_2_joint",
    "robot_arm_wrist_3_joint",
]
ARM_CMD_TOPIC = "/robot/arm/pos_traj_controller/command"
ARM_INITIAL_POS = [-0.785, -0.785, -2.000, 0.000, 0.000, 0.000]

# Gripper config
GRIPPER_LINK = "robot_arm_wrist_3_link"
GRIPPER_JOINT = "robot_egh_gripper_finger_left_joint"
GRIPPER_CMD_TOPIC = "/robot/gripper/pos_traj_controller/command"
GRIPPER_CLOSE_POS = 0.035


def close_gripper(pub):
    cmd = JointTrajectory()
    cmd.joint_names = [GRIPPER_JOINT]

    point = JointTrajectoryPoint()
    point.positions = [GRIPPER_CLOSE_POS]
    point.time_from_start = rospy.Duration(2.0)
    cmd.points = [point]

    pub.publish(cmd)
    rospy.loginfo("Gripper close command sent")


def move_arm_to_initial_position(pub):
    cmd = JointTrajectory()
    cmd.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = ARM_INITIAL_POS
    point.time_from_start = rospy.Duration(3.0)
    cmd.points = [point]

    pub.publish(cmd)
    rospy.loginfo("Arm move-to-initial command sent")


def main():
    rospy.init_node("attach_object_to_gripper")

    # Wait for attach service
    rospy.wait_for_service("/link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    arm_pub = rospy.Publisher(ARM_CMD_TOPIC, JointTrajectory, queue_size=1)
    gripper_pub = rospy.Publisher(GRIPPER_CMD_TOPIC, JointTrajectory, queue_size=1)

    # Create attach request
    req = AttachRequest()
    req.model_name_1 = ROBOT_MODEL
    req.link_name_1 = GRIPPER_LINK
    req.model_name_2 = ORANGE_MODEL
    req.link_name_2 = ORANGE_LINK

    try:
        if attach_srv(req).ok:
            rospy.loginfo("Attached %s::%s to %s::%s", ORANGE_MODEL, ORANGE_LINK, ROBOT_MODEL, GRIPPER_LINK)
            rospy.sleep(10.0)
            close_gripper(gripper_pub)
            rospy.sleep(5.0)
            move_arm_to_initial_position(arm_pub)
        else:
            rospy.logwarn("Attach failed")
    except rospy.ServiceException as exc:
        rospy.logerr("Attach service failed: %s", str(exc))


if __name__ == "__main__":
    main()
