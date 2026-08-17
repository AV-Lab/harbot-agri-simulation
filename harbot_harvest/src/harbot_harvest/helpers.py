"""Shared ROS naming and interface helpers for HARBOT harvest nodes."""

import rospy


DEFAULT_TOPICS = {
    "cmd_vel_topic": "robotnik_base_control/cmd_vel",
    "odom_topic": "robotnik_base_control/odom_gt",
    "result_topic": "orange_action",
    "status_topic": "path_nav/status",
    "proceed_flag_topic": "path_nav/proceed_flag",
}

ROBOT_NAMESPACE_ALIASES = {
    "a": "/robot",
    "b": "/robot_b",
}


def normalize_robot_namespace(robot_namespace):
    """Normalize a required robot namespace and expand A/B aliases."""
    cleaned = str(robot_namespace).strip().strip("/")
    if not cleaned:
        raise ValueError("Robot namespace cannot be empty.")

    alias = ROBOT_NAMESPACE_ALIASES.get(cleaned.lower())
    return alias if alias is not None else "/" + cleaned


def resolve_ros_name(robot_namespace, name):
    """Resolve a relative ROS interface name below a namespace."""
    cleaned_name = str(name).strip()
    if not cleaned_name:
        raise ValueError("ROS interface name cannot be empty")
    if cleaned_name.startswith("/"):
        return "/" + cleaned_name.strip("/")

    return "{}/{}".format(
        normalize_robot_namespace(robot_namespace),
        cleaned_name.strip("/"),
    )


def get_topic_setting(parameter_name):
    """Read one supported topic name from the current node's parameters."""
    if parameter_name not in DEFAULT_TOPICS:
        raise ValueError("Unknown interface parameter '{}'.".format(parameter_name))

    configured_value = rospy.get_param(
        "~{}".format(parameter_name),
        DEFAULT_TOPICS[parameter_name],
    )
    cleaned_value = str(configured_value).strip()
    if not cleaned_value:
        raise ValueError("~{} cannot be empty.".format(parameter_name))
    return cleaned_value


def get_robot_topic(robot_namespace, parameter_name):
    """Return the resolved topic name for a robot."""
    return resolve_ros_name(
        robot_namespace,
        get_topic_setting(parameter_name),
    )
