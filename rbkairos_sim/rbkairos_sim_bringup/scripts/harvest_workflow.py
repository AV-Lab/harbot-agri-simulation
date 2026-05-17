#!/usr/bin/env python3

import os
import sys
import time
import math
import json
import rospy
import signal
import argparse
import subprocess
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


WAYPOINTS = {
    "robot_a": {
        "w1": (-7.5, -12.0, 1.5708),
        "w2": (-7.5, -8.559429, 1.5708),
        "w3": (-7.5, -5.118858, 1.5708),
        "w4": (-7.5, -1.678287, 1.5708),
        "w5": (-7.5, 1.762284, 1.5708),
        "w6": (-7.5, 5.202855, 1.5708),
        "w7": (-7.5, 8.643426, 1.5708),
        "w8": (-7.5, 12.083997, 1.5708),
        "w9": (-4.0, 12.083997, -1.5708),
        "w10": (-4.0, 8.643426, -1.5708),
        "w11": (-4.0, 5.202855, -1.5708),
        "w12": (-4.0, 1.762284, -1.5708),
        "w13": (-4.0, -1.678287, -1.5708),
        "w14": (-4.0, -5.118858, -1.5708),
        "w15": (-4.0, -8.559429, -1.5708),
        "w16": (-4.0, -12.0, -1.5708),
        "w17": (0.0, -12.0, 1.5708),
        "w18": (0.0, -8.559429, 1.5708),
        "w19": (0.0, -5.118858, 1.5708),
        "w20": (0.0, -1.678287, 1.5708),
    },
    "robot_b": {
        "w21": (0.0, 1.762284, -1.5708),
        "w22": (0.0, 5.202855, -1.5708),
        "w23": (0.0, 8.643426, -1.5708),
        "w24": (0.0, 12.083997, -1.5708),
        "w25": (4.0, 12.083997, 1.5708),
        "w26": (4.0, 8.643426, 1.5708),
        "w27": (4.0, 5.202855, 1.5708),
        "w28": (4.0, 1.762284, 1.5708),
        "w29": (4.0, -1.678287, 1.5708),
        "w30": (4.0, -5.118858, 1.5708),
        "w31": (4.0, -8.559429, 1.5708),
        "w32": (4.0, -12.0, 1.5708),
        "w33": (7.5, -12.0, -1.5708),
        "w34": (7.5, -8.559429, -1.5708),
        "w35": (7.5, -5.118858, -1.5708),
        "w36": (7.5, -1.678287, -1.5708),
        "w37": (7.5, 1.762284, -1.5708),
        "w38": (7.5, 5.202855, -1.5708),
        "w39": (7.5, 8.643426, -1.5708),
        "w40": (7.5, 12.083997, -1.5708),
    },
}

FRUIT_OPTIONS = {
    "w1": ["orange1.1.L.B", "orange1.1.L.T"],
    "w2": ["orange1.2.L.B", "orange1.2.L.T"],
    "w3": ["orange1.3.L.B", "orange1.3.L.T"],
    "w4": ["orange1.4.L.B", "orange1.4.L.T"],
    "w5": ["orange1.5.L.B", "orange1.5.L.T"],
    "w6": ["orange1.6.L.B", "orange1.6.L.T"],
    "w7": ["orange1.7.L.B", "orange1.7.L.T"],
    "w8": ["orange1.8.L.B", "orange1.8.L.T"],
    "w9": ["orange1.8.R.B", "orange1.8.R.T", "orange2.8.L.B", "orange2.8.L.T"],
    "w10": ["orange1.7.R.B", "orange1.7.R.T", "orange2.7.L.B", "orange2.7.L.T"],
    "w11": ["orange1.6.R.B", "orange1.6.R.T", "orange2.6.L.B", "orange2.6.L.T"],
    "w12": ["orange1.5.R.B", "orange1.5.R.T", "orange2.5.L.B", "orange2.5.L.T"],
    "w13": ["orange1.4.R.B", "orange1.4.R.T", "orange2.4.L.B", "orange2.4.L.T"],
    "w14": ["orange1.3.R.B", "orange1.3.R.T", "orange2.3.L.B", "orange2.3.L.T"],
    "w15": ["orange1.2.R.B", "orange1.2.R.T", "orange2.2.L.B", "orange2.2.L.T"],
    "w16": ["orange1.1.R.B", "orange1.1.R.T", "orange2.1.L.B", "orange2.1.L.T"],
    "w17": ["orange2.1.R.B", "orange2.1.R.T", "orange3.1.L.B", "orange3.1.L.T"],
    "w18": ["orange2.2.R.B", "orange2.2.R.T", "orange3.2.L.B", "orange3.2.L.T"],
    "w19": ["orange2.3.R.B", "orange2.3.R.T", "orange3.3.L.B", "orange3.3.L.T"],
    "w20": ["orange2.4.R.B", "orange2.4.R.T", "orange3.4.L.B", "orange3.4.L.T"],
    "w21": ["orange2.5.R.B", "orange2.5.R.T", "orange3.5.L.B", "orange3.5.L.T"],
    "w22": ["orange2.6.R.B", "orange2.6.R.T", "orange3.6.L.B", "orange3.6.L.T"],
    "w23": ["orange2.7.R.B", "orange2.7.R.T", "orange3.7.L.B", "orange3.7.L.T"],
    "w24": ["orange2.8.R.B", "orange2.8.R.T", "orange3.8.L.B", "orange3.8.L.T"],
    "w25": ["orange3.8.R.B", "orange3.8.R.T", "orange4.8.L.B", "orange4.8.L.T"],
    "w26": ["orange3.7.R.B", "orange3.7.R.T", "orange4.7.L.B", "orange4.7.L.T"],
    "w27": ["orange3.6.R.B", "orange3.6.R.T", "orange4.6.L.B", "orange4.6.L.T"],
    "w28": ["orange3.5.R.B", "orange3.5.R.T", "orange4.5.L.B", "orange4.5.L.T"],
    "w29": ["orange3.4.R.B", "orange3.4.R.T", "orange4.4.L.B", "orange4.4.L.T"],
    "w30": ["orange3.3.R.B", "orange3.3.R.T", "orange4.3.L.B", "orange4.3.L.T"],
    "w31": ["orange3.2.R.B", "orange3.2.R.T", "orange4.2.L.B", "orange4.2.L.T"],
    "w32": ["orange3.1.R.B", "orange3.1.R.T", "orange4.1.L.B", "orange4.1.L.T"],
    "w33": ["orange4.1.R.B", "orange4.1.R.T"],
    "w34": ["orange4.2.R.B", "orange4.2.R.T"],
    "w35": ["orange4.3.R.B", "orange4.3.R.T"],
    "w36": ["orange4.4.R.B", "orange4.4.R.T"],
    "w37": ["orange4.5.R.B", "orange4.5.R.T"],
    "w38": ["orange4.6.R.B", "orange4.6.R.T"],
    "w39": ["orange4.7.R.B", "orange4.7.R.T"],
    "w40": ["orange4.8.R.B", "orange4.8.R.T"],
}

UNRIPE_ORANGES = {
    "orange1.2.L.T",
    "orange1.5.L.T",
    "orange1.8.L.T",
    "orange2.7.L.T",
    "orange1.6.R.T",
    "orange2.4.L.T",
    "orange1.3.R.T",
    "orange2.1.L.T",
    "orange2.2.R.T",
    "orange3.3.L.T",
    "orange3.6.L.T",
    "orange2.8.R.T",
    "orange3.7.R.T",
    "orange4.4.L.T",
    "orange3.1.R.T",
    "orange4.6.R.T",
}

ROBOT_ALIASES = {
    "a": "robot_a",
    "b": "robot_b",
    "robot": "robot_a",
    "robot_a": "robot_a",
    "robot_b": "robot_b",
}

UNLOADING_STATIONS = {
    "robot_a": {
        "label": "ul1",
        "points": ["ul1_1", "ul1_2"],
    },
    "robot_b": {
        "label": "ul2",
        "points": ["ul2_1", "ul2_2"],
    },
}

USE_COLOR = sys.stdout.isatty()
COLOR_TEXT = "\033[96m" if USE_COLOR else ""
COLOR_DETAIL = "\033[92m" if USE_COLOR else ""
COLOR_RESET = "\033[0m" if USE_COLOR else ""

PICK_ARM_TYPE = "Franka"
ORANGE_ACTION_TIMEOUT = 120.0
STEP_SHUTDOWN_DELAY = 2.0
INTER_ORANGE_DELAY = 2.0
PROCESS_EXIT_GRACE_DELAY = 1.0
UNLOAD_ACTION = "open"

ORANGE_ACTION_SUBSCRIBERS = {}
ORANGE_ACTION_EVENTS = {}

AUTO_TEST_ORDERS = {
    "robot_a": ["w{}".format(index) for index in range(1, 21)],
    "robot_b": ["w{}".format(index) for index in range(40, 20, -1)],
}


def info(text):
    print("{}[harvest_workflow] {}{}".format(COLOR_TEXT, text, COLOR_RESET))


def detail(text):
    print("{}  {}{}".format(COLOR_DETAIL, text, COLOR_RESET))


def prompt(text):
    return input("{}[harvest_workflow] {}{}".format(COLOR_TEXT, text, COLOR_RESET))


def orange_action_callback_factory(robot_ns):
    def _callback(message):
        ORANGE_ACTION_EVENTS[robot_ns] = {
            "value": message.data,
            "time": time.monotonic(),
        }
    return _callback


def ensure_orange_action_subscriber(robot_ns):
    if robot_ns in ORANGE_ACTION_SUBSCRIBERS:
        return

    topic_name = "{}/orange_action".format(robot_ns)
    ORANGE_ACTION_SUBSCRIBERS[robot_ns] = rospy.Subscriber(
        topic_name,
        Bool,
        orange_action_callback_factory(robot_ns),
        queue_size=10,
    )


def waypoint_sort_key(waypoint_name):
    return int(waypoint_name[1:]) if waypoint_name.startswith("w") and waypoint_name[1:].isdigit() else waypoint_name


def destination_sort_key(robot_key, destination_name):
    unloading_label = UNLOADING_STATIONS[robot_key]["label"]
    if destination_name.startswith("w") and destination_name[1:].isdigit():
        return (0, int(destination_name[1:]))
    if destination_name == unloading_label:
        return (1, 0)
    return (2, destination_name)


def normalize_namespace(robot_ns):
    cleaned = robot_ns.strip()
    if not cleaned:
        raise ValueError("Robot namespace cannot be empty.")
    lowered = cleaned.lower()
    if lowered == "a":
        return "/robot"
    if lowered == "b":
        return "/robot_b"
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    return cleaned.rstrip("/")


def namespace_to_path_key(robot_ns):
    base_name = robot_ns.strip("/").lower()
    if base_name not in ROBOT_ALIASES:
        raise ValueError(
            "Unsupported robot namespace '{}'. Use a, b, /robot, or /robot_b.".format(robot_ns)
        )
    return ROBOT_ALIASES[base_name]


def prompt_robot_namespace(default_ns="/robot"):
    entered = prompt(
        "Robot [a/b]: "
    ).strip()
    return normalize_namespace(entered or "a")


def prompt_gui_flag(default_value=False):
    while True:
        entered = prompt(
            "Launch path_nav GUI? [y/N]: " if not default_value else "Launch path_nav GUI? [Y/n]: "
        ).strip().lower()
        if not entered:
            return default_value
        if entered in ("y", "yes"):
            return True
        if entered in ("n", "no"):
            return False
        info("Please enter y or n.")


def angle_diff(angle_a, angle_b):
    delta = angle_a - angle_b
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return abs(delta)


def pose_from_odom(message):
    quat = message.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        yaw,
    )


def get_current_pose(robot_ns, timeout=5.0):
    odom_topic = "{}/robotnik_base_control/odom_gt".format(robot_ns)
    message = rospy.wait_for_message(odom_topic, Odometry, timeout=timeout)
    return pose_from_odom(message)


def nearest_waypoint(robot_key, pose):
    best_name = None
    best_score = None
    for waypoint_name, waypoint_pose in WAYPOINTS[robot_key].items():
        distance = math.hypot(pose[0] - waypoint_pose[0], pose[1] - waypoint_pose[1])
        yaw_error = angle_diff(pose[2], waypoint_pose[2])
        score = distance + 0.25 * yaw_error
        if best_score is None or score < best_score:
            best_score = score
            best_name = waypoint_name
    return best_name, best_score


def rospack_find(package_name):
    output = subprocess.check_output(["rospack", "find", package_name], text=True)
    return output.strip()


def path_directory(robot_key, node_name):
    package_path = rospack_find("rbkairos_path_nav")
    if node_name.startswith("w") and node_name[1:].isdigit():
        return os.path.join(
            package_path,
            "paths",
            robot_key,
            "wavepoint{}".format(node_name[1:]),
        )
    return os.path.join(package_path, "paths", robot_key, node_name)


def loading_station_label(robot_key):
    return UNLOADING_STATIONS[robot_key]["label"]


def is_unloading_point(robot_key, node_name):
    return node_name in UNLOADING_STATIONS[robot_key]["points"]


def display_node_name(robot_key, node_name):
    if is_unloading_point(robot_key, node_name):
        return loading_station_label(robot_key)
    return node_name


def list_raw_destinations(robot_key, current_node):
    directory = path_directory(robot_key, current_node)
    if not os.path.isdir(directory):
        return []

    destinations = []
    prefix = "{}-".format(current_node)
    for file_name in sorted(os.listdir(directory)):
        if not file_name.endswith(".json") or not file_name.startswith(prefix):
            continue
        destination = file_name[len(prefix):-5]
        if destination in WAYPOINTS[robot_key] or is_unloading_point(robot_key, destination):
            destinations.append(destination)
    return destinations


def available_destinations(robot_key, current_node):
    raw_destinations = list_raw_destinations(robot_key, current_node)
    displayed_destinations = []
    seen = set()

    for destination in raw_destinations:
        shown_name = display_node_name(robot_key, destination)
        if shown_name in seen:
            continue
        seen.add(shown_name)
        displayed_destinations.append(shown_name)

    return sorted(displayed_destinations, key=lambda name: destination_sort_key(robot_key, name))


def resolve_destination_node(robot_key, current_node, selected_destination):
    raw_destinations = list_raw_destinations(robot_key, current_node)
    unloading_label = loading_station_label(robot_key)
    if selected_destination != unloading_label:
        return selected_destination

    unloading_points = set(UNLOADING_STATIONS[robot_key]["points"])
    for destination in sorted(raw_destinations):
        if destination in unloading_points:
            return destination
    return selected_destination


def read_path_points(file_path):
    with open(file_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def unload_point_pose(robot_key, unload_point):
    directory = path_directory(robot_key, unload_point)
    if not os.path.isdir(directory):
        return None

    json_files = sorted(
        file_name for file_name in os.listdir(directory) if file_name.endswith(".json")
    )
    if not json_files:
        return None

    points = read_path_points(os.path.join(directory, json_files[0]))
    if not points:
        return None

    pose = points[0]
    return (float(pose[0]), float(pose[1]), float(pose[2]))


def nearest_named_pose(candidates, pose):
    best_name = None
    best_score = None
    for candidate_name, candidate_pose in candidates.items():
        distance = math.hypot(pose[0] - candidate_pose[0], pose[1] - candidate_pose[1])
        yaw_error = angle_diff(pose[2], candidate_pose[2])
        score = distance + 0.25 * yaw_error
        if best_score is None or score < best_score:
            best_score = score
            best_name = candidate_name
    return best_name, best_score


def detect_current_node(robot_key, pose):
    waypoint_name, waypoint_score = nearest_named_pose(WAYPOINTS[robot_key], pose)

    unloading_candidates = {}
    for unload_point in UNLOADING_STATIONS[robot_key]["points"]:
        unload_pose = unload_point_pose(robot_key, unload_point)
        if unload_pose is not None:
            unloading_candidates[unload_point] = unload_pose

    if not unloading_candidates:
        return waypoint_name, waypoint_score

    unload_name, unload_score = nearest_named_pose(unloading_candidates, pose)
    if unload_score < waypoint_score:
        return unload_name, unload_score
    return waypoint_name, waypoint_score


def prompt_destination(destinations):
    info("Available destinations:")
    for index, destination_name in enumerate(destinations, start=1):
        detail("{}. {}".format(index, destination_name))
    detail("0. Exit")

    while True:
        selected = prompt("Select destination number: ").strip()
        if selected == "0":
            return None
        if not selected.isdigit():
            info("Please enter a number from the list.")
            continue
        selected_index = int(selected)
        if 1 <= selected_index <= len(destinations):
            return destinations[selected_index - 1]
        info("Please enter a valid option number.")


def build_path_name(robot_key, start_node, destination_node):
    if start_node.startswith("w") and start_node[1:].isdigit():
        return "{}/wavepoint{}/{}-{}.json".format(
            robot_key,
            start_node[1:],
            start_node,
            destination_node,
        )
    return "{}/{}/{}-{}.json".format(
        robot_key,
        start_node,
        start_node,
        destination_node,
    )


def launch_navigation(robot_ns, path_name, gui_enabled):
    command = [
        "roslaunch",
        "rbkairos_path_nav",
        "path_nav.launch",
        "path_name:={}".format(path_name),
        "robot_ns:={}".format(robot_ns),
        "gui:={}".format(str(gui_enabled).lower()),
    ]

    return subprocess.Popen(
        command,
        preexec_fn=os.setsid,
        stdout=None,
        stderr=None,
    )


def launch_persistent_path_gui(robot_ns):
    command = [
        "rosrun",
        "rbkairos_path_nav",
        "path_gui.py",
        "__ns:={}".format(robot_ns),
        "_odom_topic:=robotnik_base_control/odom_gt",
        "_global_path_topic:=path_nav/path",
        "_desired_velocity_topic:=path_nav/desired_speed",
        "_state_flag_topic:=path_nav/status",
        "_proceed_flag_topic:=path_nav/proceed_flag",
        "_reverse_mode_topic:=path_nav/reverse_mode",
    ]

    return subprocess.Popen(
        command,
        preexec_fn=os.setsid,
        stdout=None,
        stderr=None,
    )


def launch_ros_script(script_name, private_params):
    command = [
        "rosrun",
        "rbkairos_sim_bringup",
        script_name,
    ]
    for param_name, param_value in private_params.items():
        command.append("_{}:={}".format(param_name, param_value))

    return subprocess.Popen(
        command,
        preexec_fn=os.setsid,
        stdout=None,
        stderr=None,
    )


def shutdown_process(process_handle):
    if process_handle is None or process_handle.poll() is not None:
        return

    pgid = os.getpgid(process_handle.pid)
    os.killpg(pgid, signal.SIGINT)

    deadline = time.time() + 5.0
    while time.time() < deadline:
        if process_handle.poll() is not None:
            return
        time.sleep(0.1)

    os.killpg(pgid, signal.SIGTERM)


def wait_for_navigation_finish(robot_ns):
    status_topic = "{}/path_nav/status".format(robot_ns)
    last_status = None
    while not rospy.is_shutdown():
        message = rospy.wait_for_message(status_topic, String, timeout=2.0)
        if message.data != last_status:
            info("path_nav status: {}".format(message.data))
            last_status = message.data
        if message.data == "FINISHED":
            return


def send_proceed_flag(robot_ns):
    topic_name = "{}/path_nav/proceed_flag".format(robot_ns)
    publisher = rospy.Publisher(topic_name, Bool, queue_size=1, latch=True)

    deadline = time.time() + 3.0
    while publisher.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.1)

    publisher.publish(Bool(data=True))


def wait_for_orange_action_true(robot_ns, process_handle, step_label, timeout=ORANGE_ACTION_TIMEOUT):
    ensure_orange_action_subscriber(robot_ns)
    start_time = time.monotonic()
    deadline = time.time() + timeout
    process_exit_time = None

    while not rospy.is_shutdown():
        latest_event = ORANGE_ACTION_EVENTS.get(robot_ns)
        if latest_event and latest_event["value"] and latest_event["time"] >= start_time:
            # info("{} reported orange_action=True.".format(step_label))
            return

        if process_handle.poll() is not None:
            if process_exit_time is None:
                process_exit_time = time.monotonic()
            elif time.monotonic() - process_exit_time >= PROCESS_EXIT_GRACE_DELAY:
                raise RuntimeError("{} exited before publishing orange_action.".format(step_label))

        remaining_time = deadline - time.time()
        if remaining_time <= 0.0:
            raise RuntimeError(
                "Timed out waiting for orange_action from {} after {:.1f} seconds.".format(
                    step_label,
                    timeout,
                )
            )

        rospy.sleep(min(0.1, remaining_time))


def run_orange_script_step(robot_ns, orange_name, step_label, script_name, extra_params=None):
    private_params = {
        "orange": orange_name,
    }
    if extra_params:
        private_params.update(extra_params)

    info(step_label)
    process_handle = None

    try:
        process_handle = launch_ros_script(script_name, private_params)
        wait_for_orange_action_true(robot_ns, process_handle, step_label)
        rospy.sleep(STEP_SHUTDOWN_DELAY)
    finally:
        shutdown_process(process_handle)


def execute_orange_pick(robot_ns, orange_name):
    run_orange_script_step(
        robot_ns,
        orange_name,
        "Step 1/4: Approach target fruit",
        "approach_orange.py",
        {
            "robot_ns": robot_ns,
        },
    )
    run_orange_script_step(
        robot_ns,
        orange_name,
        "Step 2/4: Reach fruit with robotic arm",
        "moveit_target_reach.py",
        {
            "robot_ns": robot_ns,
            "arm": PICK_ARM_TYPE,
        },
    )
    run_orange_script_step(
        robot_ns,
        orange_name,
        "Step 3/4: Grasp and retract fruit",
        "grasp_orange.py",
        {
            "robot_ns": robot_ns,
            "arm": PICK_ARM_TYPE,
        },
    )
    run_orange_script_step(
        robot_ns,
        orange_name,
        "Step 4/4: Unload fruit from gripper",
        "unload_orange.py",
        {
            "robot_ns": robot_ns,
        },
    )
    info("Resetting environment.")


def execute_fruit_sequence(robot_ns, selected_fruits):
    if not selected_fruits:
        return True

    for index, orange_name in enumerate(selected_fruits, start=1):
        if orange_name in UNRIPE_ORANGES:
            info("{} is unripe. Skipping harvest.".format(orange_name))
            continue

        info("Executing complete harvesting workflow for {}.".format(orange_name))
        try:
            execute_orange_pick(robot_ns, orange_name)
        except (OSError, RuntimeError, rospy.ROSException) as exc:
            info("Harvesting workflow failed for {}: {}".format(orange_name, exc))
            return False

        info("Completed harvesting workflow for {}.".format(orange_name))
        rospy.sleep(INTER_ORANGE_DELAY)

    return True


def show_fruit_options(location_name):
    fruits = FRUIT_OPTIONS.get(location_name, [])
    info("Fruit options at {}:".format(location_name))
    if not fruits:
        detail("No fruit options configured yet.")
        return []

    for index, fruit_name in enumerate(fruits, start=1):
        detail("{}. {}".format(index, fruit_name))
    return fruits


def prompt_fruit_selection(fruits):
    if not fruits:
        prompt("Press Enter to continue...")
        return []

    info("Enter fruit numbers separated by spaces or commas, or press Enter to skip.")
    while True:
        selected = prompt("Fruit selection: ").strip()
        if not selected:
            return []

        tokens = selected.replace(",", " ").split()
        if not all(token.isdigit() for token in tokens):
            info("Please enter only numbers from the list.")
            continue

        indexes = []
        valid = True
        for token in tokens:
            index = int(token)
            if index < 1 or index > len(fruits):
                valid = False
                break
            indexes.append(index)
        if not valid:
            info("One or more fruit numbers are out of range.")
            continue

        chosen = [fruits[index - 1] for index in indexes]
        info("Selected fruits: {}".format(", ".join(chosen)))
        return chosen


def prompt_navigation_choice(current_location_name):
    while True:
        choice = prompt(
            "Current location is {}. Navigate to another destination? [Y/n]: ".format(current_location_name)
        ).strip().lower()
        if choice in ("", "y", "yes"):
            return True
        if choice in ("n", "no"):
            return False
        info("Please enter y or n.")


def prompt_full_automatic_test():
    while True:
        choice = prompt("Run full automatic test from the current waypoint? [y/N]: ").strip().lower()
        if choice in ("", "n", "no"):
            return False
        if choice in ("y", "yes"):
            return True
        info("Please enter y or n.")


def handle_fruit_flow(robot_ns, location_name):
    if location_name not in FRUIT_OPTIONS:
        return

    fruits = show_fruit_options(location_name)
    selected_fruits = prompt_fruit_selection(fruits)
    if selected_fruits:
        execute_fruit_sequence(robot_ns, selected_fruits)


def handle_fruit_flow_automatic(robot_ns, location_name):
    fruits = FRUIT_OPTIONS.get(location_name, [])
    if not fruits:
        info("No fruit options configured at {}.".format(location_name))
        return True

    info("Automatic fruit collection at {}: {}.".format(location_name, ", ".join(fruits)))
    return execute_fruit_sequence(robot_ns, fruits)


def navigate_to_node(robot_ns, robot_key, start_node, destination_node, gui_enabled):
    path_name = build_path_name(robot_key, start_node, destination_node)
    info("Launching path_nav with path_name={} ...".format(path_name))

    nav_process = None
    try:
        nav_process = launch_navigation(robot_ns, path_name, gui_enabled)
        rospy.sleep(5.0)
        send_proceed_flag(robot_ns)
        wait_for_navigation_finish(robot_ns)
        rospy.sleep(3.0)
    finally:
        shutdown_process(nav_process)

    updated_pose = get_current_pose(robot_ns)
    reached_node, reached_score = detect_current_node(robot_key, updated_pose)
    reached_location_name = display_node_name(robot_key, reached_node)
    info(
        "Robot is now nearest to {} (match score {:.3f}).".format(
            reached_location_name, reached_score
        )
    )
    return reached_node


def run_container_unload(robot_ns):
    info("Running container unload action: {}.".format(UNLOAD_ACTION))
    process_handle = None
    try:
        process_handle = launch_ros_script(
            "container_control.py",
            {
                "robot_ns": robot_ns,
                "action": UNLOAD_ACTION,
            },
        )
        return_code = process_handle.wait()
        if return_code != 0:
            info("container_control.py exited with code {}.".format(return_code))
            return False
    finally:
        shutdown_process(process_handle)

    info("Container unload action completed.")
    return True


def navigate_to_unloading_station(robot_ns, robot_key, active_node, gui_enabled):
    unloading_label = loading_station_label(robot_key)
    destination_node = resolve_destination_node(robot_key, active_node, unloading_label)

    if destination_node == unloading_label:
        info("No unloading path found from {} to {}.".format(active_node, unloading_label))
        return None

    info("Automatic test finished waypoint collection. Going to {}.".format(unloading_label))
    try:
        reached_node = navigate_to_node(robot_ns, robot_key, active_node, destination_node, gui_enabled)
    except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
        info("Automatic navigation to {} failed: {}".format(unloading_label, exc))
        return None

    if not is_unloading_point(robot_key, reached_node):
        info("Expected to reach {}, but detected {} instead.".format(unloading_label, display_node_name(robot_key, reached_node)))
        return None

    return reached_node


def run_full_automatic_test(robot_ns, robot_key, current_node, gui_enabled):
    if current_node not in WAYPOINTS[robot_key]:
        info("Full automatic test can only start from a waypoint, not from {}.".format(display_node_name(robot_key, current_node)))
        return False

    waypoint_order = AUTO_TEST_ORDERS[robot_key]
    if current_node not in waypoint_order:
        info("Current waypoint {} is not part of the automatic test order.".format(current_node))
        return False

    start_index = waypoint_order.index(current_node)
    remaining_waypoints = waypoint_order[start_index:]
    info("Starting full automatic test from {}.".format(current_node))

    if not handle_fruit_flow_automatic(robot_ns, current_node):
        return False

    active_node = current_node
    for next_waypoint in remaining_waypoints[1:]:
        try:
            active_node = navigate_to_node(robot_ns, robot_key, active_node, next_waypoint, gui_enabled)
        except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
            info("Automatic navigation failed while going to {}: {}".format(next_waypoint, exc))
            return False

        if active_node != next_waypoint:
            info("Expected to reach {}, but detected {} instead.".format(next_waypoint, display_node_name(robot_key, active_node)))
            return False

        if not handle_fruit_flow_automatic(robot_ns, active_node):
            return False

    unload_node = navigate_to_unloading_station(robot_ns, robot_key, active_node, gui_enabled)
    if unload_node is None:
        return False

    if not run_container_unload(robot_ns):
        return False

    info("Full automatic test completed.")
    return True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive waypoint navigator for RBKairos simulation."
    )
    parser.add_argument(
        "--robot-ns",
        help="Robot namespace, for example /robot or /robot_b.",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch path_nav with GUI enabled.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    rospy.init_node("harvest_workflow", anonymous=True)
    arguments = parse_args()

    try:
        robot_ns = normalize_namespace(arguments.robot_ns) if arguments.robot_ns else prompt_robot_namespace()
        robot_key = namespace_to_path_key(robot_ns)
    except ValueError as exc:
        info(str(exc))
        return 1

    gui_requested = arguments.gui or prompt_gui_flag(default_value=False)
    auto_test_prompt_pending = True
    path_gui_process = None
    path_launch_gui_enabled = False

    try:
        if gui_requested:
            path_gui_process = launch_persistent_path_gui(robot_ns)
            info("Persistent path GUI started for {}.".format(robot_ns))
            rospy.sleep(1.0)

        info("Listening to {}...".format(robot_ns))

        while not rospy.is_shutdown():
            try:
                current_pose = get_current_pose(robot_ns)
            except rospy.ROSException as exc:
                info("Failed to read odometry from {}: {}".format(robot_ns, exc))
                return 1

            current_node, score = detect_current_node(robot_key, current_pose)
            current_location_name = display_node_name(robot_key, current_node)
            info(
                "Current pose is nearest to {} (match score {:.3f}).".format(current_location_name, score)
            )

            if auto_test_prompt_pending:
                auto_test_prompt_pending = False
                if prompt_full_automatic_test():
                    if run_full_automatic_test(robot_ns, robot_key, current_node, path_launch_gui_enabled):
                        return 0
                    info("Automatic test did not complete.")
                    return 1

            if not prompt_navigation_choice(current_location_name):
                handle_fruit_flow(robot_ns, current_node)
                continue_choice = prompt("Stay in the script and choose another destination? [Y/n]: ").strip().lower()
                if continue_choice in ("n", "no"):
                    info("Exiting interactive waypoint navigator.")
                    return 0
                continue

            destinations = available_destinations(robot_key, current_node)
            if not destinations:
                info("No path files found for {} from {}.".format(robot_key, current_location_name))
                return 1

            destination_waypoint = prompt_destination(destinations)
            if destination_waypoint is None:
                info("Exiting interactive waypoint navigator.")
                return 0

            destination_node = resolve_destination_node(robot_key, current_node, destination_waypoint)
            try:
                reached_node = navigate_to_node(robot_ns, robot_key, current_node, destination_node, path_launch_gui_enabled)
            except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
                info("Navigation failed: {}".format(exc))
                return 1
            handle_fruit_flow(robot_ns, reached_node)

        return 0
    finally:
        shutdown_process(path_gui_process)


if __name__ == "__main__":
    sys.exit(main())
