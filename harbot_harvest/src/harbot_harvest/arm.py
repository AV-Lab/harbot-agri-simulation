# ******************************************************************************
# Code Summary:
# - Describe the Franka arm: joints, links, gripper, and fixed poses.
# - Build the joint and link names for the selected robot namespace.
# - Build single-point joint trajectory commands for the arm and gripper.
# - Connect to a MoveIt move group, retrying when the connection fails.
# ******************************************************************************


import rospy
from collections import namedtuple
from harbot_harvest import helpers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# "{prefix}" is replaced by the robot's joint prefix, robot or robot_b.
FRANKA_SETTINGS = {
    "move_group": "arm",
    "gripper_open": 0.5,
    "gripper_close": -0.1,
    "ee_link": "{prefix}_hand_tcp",
    "gripper_joint": "{prefix}_finger_joint1",
    "home_pose": [0.000, -0.784, 0.000, -2.100, 0.000, 1.569, 0.782],
    "drop_pose": [0.001, 0.615, -0.000, -1.754, 0.000, 2.463, 0.785],
    "arm_joints": ["{prefix}_joint1", "{prefix}_joint2", "{prefix}_joint3", "{prefix}_joint4", "{prefix}_joint5", "{prefix}_joint6", "{prefix}_joint7"],
}

ArmConfig = namedtuple("ArmConfig", FRANKA_SETTINGS)


def fill_prefix(value, prefix):
    if isinstance(value, str):
        return value.format(prefix=prefix)
    if isinstance(value, list):
        return [fill_prefix(item, prefix) for item in value]
    return value


def get_arm_config(robot_namespace):
    prefix = helpers.joint_prefix(robot_namespace)
    return ArmConfig(**{name: fill_prefix(value, prefix) for name, value in FRANKA_SETTINGS.items()})


def joint_command(joint_names, positions, duration):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)

    command = JointTrajectory()
    command.joint_names = joint_names
    command.points = [point]
    return command


def create_move_group(group_name, robot_namespace, wait_timeout, retries=0, retry_delay=0.0):
    import moveit_commander

    last_error = None
    for attempt in range(1, retries + 2):
        try:
            robot_description = robot_namespace + "/robot_description"
            return moveit_commander.MoveGroupCommander(group_name, ns=robot_namespace, wait_for_servers=wait_timeout, robot_description=robot_description)
        except RuntimeError as exc:
            last_error = exc
            rospy.logwarn("MoveGroupCommander connection failed on attempt %d/%d: %s", attempt, retries + 1, exc)
            if attempt <= retries and not rospy.is_shutdown():
                rospy.sleep(retry_delay)

    raise last_error
