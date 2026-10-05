# ******************************************************************************
# Code Summary:
# - Hold the default topic and frame settings shared by the harvest nodes.
# - Normalize robot namespaces and expand the a/b aliases.
# - Resolve relative ROS names below a robot namespace.
# - Convert between orange model names and their row, tree, side, and height.
# ******************************************************************************


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
    "a": "robot_a",
    "b": "robot_b",
    "robot": "robot_a",
    "robot_a": "robot_a",
    "robot_b": "robot_b",
}


def normalize_robot_namespace(robot_namespace):
    cleaned = str(robot_namespace).strip().strip("/")
    if not cleaned:
        raise ValueError("Robot namespace cannot be empty.")

    return ROBOT_NAMESPACE_ALIASES.get(cleaned.lower(), "/" + cleaned)


def robot_key(robot_namespace):
    cleaned = str(robot_namespace).strip("/").lower()
    if cleaned not in ROBOT_KEYS:
        raise ValueError("Unsupported robot namespace '{}'. Use a, b, /robot, or /robot_b.".format(robot_namespace))

    return ROBOT_KEYS[cleaned]


def joint_prefix(robot_namespace):
    return "robot_b" if normalize_robot_namespace(robot_namespace) == "/robot_b" else "robot"


def resolve_ros_name(robot_namespace, name):
    cleaned_name = str(name).strip()
    if not cleaned_name:
        raise ValueError("ROS interface name cannot be empty")
    if cleaned_name.startswith("/"):
        return "/" + cleaned_name.strip("/")

    return "{}/{}".format(normalize_robot_namespace(robot_namespace), cleaned_name.strip("/"))


def get_setting(parameter_name):
    if parameter_name not in DEFAULT_SETTINGS:
        raise ValueError("Unknown interface parameter '{}'.".format(parameter_name))

    value = str(rospy.get_param("~" + parameter_name, DEFAULT_SETTINGS[parameter_name])).strip()
    if not value:
        raise ValueError("~{} cannot be empty.".format(parameter_name))

    return value


def get_robot_topic(robot_namespace, parameter_name):
    return resolve_ros_name(robot_namespace, get_setting(parameter_name))


def parse_orange(orange_name):
    # Example: orange2.7.R.B -> (2, 7, "R", "B")
    parts = str(orange_name).split(".")
    if len(parts) != 4 or not parts[0].startswith("orange"):
        raise ValueError("Invalid orange name: {}".format(orange_name))

    try:
        row = int(parts[0][len("orange"):])
        tree = int(parts[1])
    except ValueError:
        raise ValueError("Invalid orange name: {}".format(orange_name)) from None

    side = parts[2].upper()
    height = parts[3].upper()
    if not 1 <= row <= 4 or not 1 <= tree <= 8 or side not in ("L", "R") or height not in ("B", "T"):
        raise ValueError("Invalid orange name: {}".format(orange_name))

    return row, tree, side, height


def format_orange(row, tree, side, height):
    return "orange{}.{}.{}.{}".format(row, tree, side, height)
