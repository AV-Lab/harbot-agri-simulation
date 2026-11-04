# ******************************************************************************
# Code Summary:
# - Hold the default topic and frame settings shared by the harvest nodes.
# - Normalize robot namespaces and expand the a/b aliases.
# - Resolve relative ROS names below a robot namespace.
# - Split an orange model name into its row, tree, side, and height.
# ******************************************************************************


import re
import rospy


DEFAULT_SETTINGS = {
    "planning_frame": "world",
    "result_topic": "orange_action",
    "status_topic": "path_nav/status",
    "proceed_flag_topic": "path_nav/proceed_flag",
    "odom_topic": "robotnik_base_control/odom_gt",
    "cmd_vel_topic": "robotnik_base_control/cmd_vel",
    "arm_command_topic": "arm/pos_traj_controller/command",
    "gripper_command_topic": "gripper/pos_traj_controller/command",
}

ROBOT_NAMESPACE_ALIASES = {
    "a": "/robot",
    "b": "/robot_b",
}

ROBOT_KEYS = {
    "/robot": "robot_a",
    "/robot_b": "robot_b",
}

ORANGE_PATTERN = re.compile(r"orange([1-4])\.([1-8])\.([LR])\.([BT])")


def normalize_robot_namespace(robot_namespace):
    cleaned = robot_namespace.strip().strip("/")
    if not cleaned:
        raise ValueError("Robot namespace cannot be empty.")

    return ROBOT_NAMESPACE_ALIASES.get(cleaned.lower(), "/" + cleaned)


def robot_key(robot_namespace):
    robot_namespace = normalize_robot_namespace(robot_namespace)
    if robot_namespace not in ROBOT_KEYS:
        raise ValueError("Unsupported robot namespace {}.".format(robot_namespace))

    return ROBOT_KEYS[robot_namespace]


def resolve_ros_name(robot_namespace, name):
    if name.startswith("/"):
        return name

    return robot_namespace + "/" + name


def get_setting(parameter_name):
    return rospy.get_param("~" + parameter_name, DEFAULT_SETTINGS[parameter_name])


def get_robot_topic(robot_namespace, parameter_name):
    return resolve_ros_name(robot_namespace, get_setting(parameter_name))


def parse_orange(orange_name):
    # Example: orange2.7.R.B -> (2, 7, "R", "B")
    match = ORANGE_PATTERN.fullmatch(orange_name)
    if not match:
        raise ValueError("Invalid orange name: {}".format(orange_name))

    row, tree, side, height = match.groups()
    return int(row), int(tree), side, height
