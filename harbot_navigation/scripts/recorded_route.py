# ******************************************************************************
# Code Summary:
# - Load the recorded route used by both the path and speed planners.
# - Find the route file from the robot namespace, start point, and route name.
# - If only a goal is given, pick the start point nearest the robot's position.
# - Check that every point has [x, y, yaw, speed, distance].
# - Return the route points as lists of floats.
# ******************************************************************************


import os
import math
import json
import glob
import rospy
import rospkg
from nav_msgs.msg import Odometry


def load_recorded_route(route):
    namespace = rospy.get_namespace().strip("/")
    robot_folder = "robot_a" if namespace == "robot" else namespace
    package_path = rospkg.RosPack().get_path("harbot_navigation")
    path_folder = rospy.get_param("~path_folder", "paths")
    route_name = os.path.splitext(route)[0]

    if "-" not in route_name:
        robot_paths = os.path.join(package_path, path_folder, robot_folder)
        start_name = find_nearest_start(robot_paths)
        if start_name == route_name:
            rospy.loginfo("Already at %s", route_name)
            return []
        route_name = "{}-{}".format(start_name, route_name)

    start_name = route_name.split("-", 1)[0]
    start_folder = (
        "wavepoint{}".format(start_name[1:])
        if start_name.startswith("a")
        else start_name
    )

    file_path = os.path.join(
        package_path, path_folder, robot_folder, start_folder, route_name + ".json"
    )
    with open(file_path, "r", encoding="utf-8") as route_file:
        points = json.load(route_file)

    route_points = []
    for index, point in enumerate(points):
        if len(point) != 5:
            raise ValueError(
                "Path point {} must contain [x, y, yaw, speed, distance]".format(index)
            )
        values = list(map(float, point))
        route_points.append(values)
    return route_points


def find_nearest_start(robot_paths):
    odom_topic = rospy.get_param("~odom_topic", "robotnik_base_control/odom_gt")
    odom = rospy.wait_for_message(odom_topic, Odometry, timeout=5.0)
    position = odom.pose.pose.position
    nearest_name = None
    nearest_distance = float("inf")

    for folder in sorted(glob.glob(os.path.join(robot_paths, "*"))):
        files = sorted(glob.glob(os.path.join(folder, "*.json")))
        if not files:
            continue

        # The first point of a recorded route gives its named start position.
        with open(files[0], "r", encoding="utf-8") as route_file:
            points = json.load(route_file)
        if not points:
            continue

        x, y = map(float, points[0][:2])
        distance = math.hypot(x - position.x, y - position.y)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_name = os.path.basename(files[0]).split("-", 1)[0]

    if nearest_name is None:
        raise ValueError("No recorded starting point found for the robot position")
    return nearest_name
