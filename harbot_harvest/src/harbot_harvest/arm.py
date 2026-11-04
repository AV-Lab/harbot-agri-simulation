# ******************************************************************************
# Code Summary:
# - Describe the Franka arm: joints, links, gripper, and fixed poses for one robot.
# - Build single-point joint trajectory commands for the arm and gripper.
# - Connect to a MoveIt move group, retrying when the connection fails.
# ******************************************************************************


import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FrankaArm:
    def __init__(self, robot_namespace):
        prefix = "robot_b" if robot_namespace == "/robot_b" else "robot"
        gripper_type = rospy.get_param(robot_namespace + "/gripper_type", "six")

        self.move_group = "arm"
        self.gripper_open = 0.5
        self.ee_link = prefix + "_hand_tcp"
        self.gripper_joint = prefix + "_finger_joint1"
        self.gripper_close = 0.21 if gripper_type == "four" else -0.1
        self.drop_pose = [0.001, 0.615, 0.000, -1.754, 0.000, 2.463, 0.785]
        self.home_pose = [0.000, -0.784, 0.000, -2.100, 0.000, 1.569, 0.782]
        self.arm_joints = ["{}_joint{}".format(prefix, index) for index in range(1, 8)]


def joint_command(joint_names, positions, duration):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)

    command = JointTrajectory()
    command.joint_names = joint_names
    command.points = [point]
    return command


def create_move_group(group_name, robot_namespace, wait_timeout, retries=0, retry_delay=0.0):
    robot_description = robot_namespace + "/robot_description"
    for attempt in range(retries + 1):
        try:
            return moveit_commander.MoveGroupCommander(group_name, ns=robot_namespace, wait_for_servers=wait_timeout, robot_description=robot_description)
        except RuntimeError as exc:
            if attempt == retries or rospy.is_shutdown():
                raise
            rospy.logwarn("MoveGroupCommander connection failed, retrying: %s", exc)
            rospy.sleep(retry_delay)
