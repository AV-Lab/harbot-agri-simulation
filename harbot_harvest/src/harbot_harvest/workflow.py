# ******************************************************************************
# Code Summary:
# - Hold the greenhouse waypoints, fruit options, and unloading stations.
# - Detect the robot's current node and list the saved routes from it.
# - Run path_nav for a route and wait until the robot reports FINISHED.
# - Run one harvest script step and wait for its orange_action result.
# - Run the container unload action.
# - Start ROS commands in their own process group and shut them down cleanly.
# - Provide the interactive harvest workflow used by harvest_workflow.py.
# - Optionally open the path GUI and the third-person camera view in RViz.
# - The PROST action server reuses these steps instead of its own copies.
# ******************************************************************************


import os
import sys
import math
import time
import rospy
import signal
import argparse
import subprocess
from nav_msgs.msg import Odometry
from harbot_harvest import helpers
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


WAYPOINTS = {
    "robot_a": {
        "a1": (-7.5, -12.0, 1.5708),
        "a2": (-7.5, -8.559429, 1.5708),
        "a3": (-7.5, -5.118858, 1.5708),
        "a4": (-7.5, -1.678287, 1.5708),
        "a5": (-7.5, 1.762284, 1.5708),
        "a6": (-7.5, 5.202855, 1.5708),
        "a7": (-7.5, 8.643426, 1.5708),
        "a8": (-7.5, 12.083997, 1.5708),
        "a16": (-4.0, 12.083997, -1.5708),
        "a15": (-4.0, 8.643426, -1.5708),
        "a14": (-4.0, 5.202855, -1.5708),
        "a13": (-4.0, 1.762284, -1.5708),
        "a12": (-4.0, -1.678287, -1.5708),
        "a11": (-4.0, -5.118858, -1.5708),
        "a10": (-4.0, -8.559429, -1.5708),
        "a9": (-4.0, -12.0, -1.5708),
        "a17": (0.0, -12.0, 1.5708),
        "a18": (0.0, -8.559429, 1.5708),
        "a19": (0.0, -5.118858, 1.5708),
        "a20": (0.0, -1.678287, 1.5708),
        "ul1_1": (-7.5, -13.6, 1.5708),
        "ul1_2": (-6.45, -14.4, 3.1416),
        "ul2_2": (6.45, 14.4, 3.1416),
    },
    "robot_b": {
        "a21": (0.0, 1.762284, -1.5708),
        "a22": (0.0, 5.202855, -1.5708),
        "a23": (0.0, 8.643426, -1.5708),
        "a24": (0.0, 12.083997, -1.5708),
        "a32": (4.0, 12.083997, 1.5708),
        "a31": (4.0, 8.643426, 1.5708),
        "a30": (4.0, 5.202855, 1.5708),
        "a29": (4.0, 1.762284, 1.5708),
        "a28": (4.0, -1.678287, 1.5708),
        "a27": (4.0, -5.118858, 1.5708),
        "a26": (4.0, -8.559429, 1.5708),
        "a25": (4.0, -12.0, 1.5708),
        "a33": (7.5, -12.0, -1.5708),
        "a34": (7.5, -8.559429, -1.5708),
        "a35": (7.5, -5.118858, -1.5708),
        "a36": (7.5, -1.678287, -1.5708),
        "a37": (7.5, 1.762284, -1.5708),
        "a38": (7.5, 5.202855, -1.5708),
        "a39": (7.5, 8.643426, -1.5708),
        "a40": (7.5, 12.083997, -1.5708),
        "ul1_2": (-6.45, -14.4, 0.0),
        "ul2_1": (7.5, 13.6, -1.5708),
        "ul2_2": (6.45, 14.4, 3.1416),
    },
}

FRUIT_OPTIONS = {
    "a1": ["orange1.1.L.B", "orange1.1.L.T"],
    "a2": ["orange1.2.L.B", "orange1.2.L.T"],
    "a3": ["orange1.3.L.B", "orange1.3.L.T"],
    "a4": ["orange1.4.L.B", "orange1.4.L.T"],
    "a5": ["orange1.5.L.B", "orange1.5.L.T"],
    "a6": ["orange1.6.L.B", "orange1.6.L.T"],
    "a7": ["orange1.7.L.B", "orange1.7.L.T"],
    "a8": ["orange1.8.L.B", "orange1.8.L.T"],
    "a16": ["orange1.8.R.B", "orange1.8.R.T", "orange2.8.L.B", "orange2.8.L.T"],
    "a15": ["orange1.7.R.B", "orange1.7.R.T", "orange2.7.L.B", "orange2.7.L.T"],
    "a14": ["orange1.6.R.B", "orange1.6.R.T", "orange2.6.L.B", "orange2.6.L.T"],
    "a13": ["orange1.5.R.B", "orange1.5.R.T", "orange2.5.L.B", "orange2.5.L.T"],
    "a12": ["orange1.4.R.B", "orange1.4.R.T", "orange2.4.L.B", "orange2.4.L.T"],
    "a11": ["orange1.3.R.B", "orange1.3.R.T", "orange2.3.L.B", "orange2.3.L.T"],
    "a10": ["orange1.2.R.B", "orange1.2.R.T", "orange2.2.L.B", "orange2.2.L.T"],
    "a9": ["orange1.1.R.B", "orange1.1.R.T", "orange2.1.L.B", "orange2.1.L.T"],
    "a17": ["orange2.1.R.B", "orange2.1.R.T", "orange3.1.L.B", "orange3.1.L.T"],
    "a18": ["orange2.2.R.B", "orange2.2.R.T", "orange3.2.L.B", "orange3.2.L.T"],
    "a19": ["orange2.3.R.B", "orange2.3.R.T", "orange3.3.L.B", "orange3.3.L.T"],
    "a20": ["orange2.4.R.B", "orange2.4.R.T", "orange3.4.L.B", "orange3.4.L.T"],
    "a21": ["orange2.5.R.B", "orange2.5.R.T", "orange3.5.L.B", "orange3.5.L.T"],
    "a22": ["orange2.6.R.B", "orange2.6.R.T", "orange3.6.L.B", "orange3.6.L.T"],
    "a23": ["orange2.7.R.B", "orange2.7.R.T", "orange3.7.L.B", "orange3.7.L.T"],
    "a24": ["orange2.8.R.B", "orange2.8.R.T", "orange3.8.L.B", "orange3.8.L.T"],
    "a32": ["orange3.8.R.B", "orange3.8.R.T", "orange4.8.L.B", "orange4.8.L.T"],
    "a31": ["orange3.7.R.B", "orange3.7.R.T", "orange4.7.L.B", "orange4.7.L.T"],
    "a30": ["orange3.6.R.B", "orange3.6.R.T", "orange4.6.L.B", "orange4.6.L.T"],
    "a29": ["orange3.5.R.B", "orange3.5.R.T", "orange4.5.L.B", "orange4.5.L.T"],
    "a28": ["orange3.4.R.B", "orange3.4.R.T", "orange4.4.L.B", "orange4.4.L.T"],
    "a27": ["orange3.3.R.B", "orange3.3.R.T", "orange4.3.L.B", "orange4.3.L.T"],
    "a26": ["orange3.2.R.B", "orange3.2.R.T", "orange4.2.L.B", "orange4.2.L.T"],
    "a25": ["orange3.1.R.B", "orange3.1.R.T", "orange4.1.L.B", "orange4.1.L.T"],
    "a33": ["orange4.1.R.B", "orange4.1.R.T"],
    "a34": ["orange4.2.R.B", "orange4.2.R.T"],
    "a35": ["orange4.3.R.B", "orange4.3.R.T"],
    "a36": ["orange4.4.R.B", "orange4.4.R.T"],
    "a37": ["orange4.5.R.B", "orange4.5.R.T"],
    "a38": ["orange4.6.R.B", "orange4.6.R.T"],
    "a39": ["orange4.7.R.B", "orange4.7.R.T"],
    "a40": ["orange4.8.R.B", "orange4.8.R.T"],
}

UNRIPE_ORANGES = {
    "orange1.2.L.T",
    "orange1.4.R.B",
    "orange1.7.L.B",
    "orange2.1.L.T",
    "orange2.5.R.T",
    "orange2.7.L.T",
    "orange3.1.R.T",
    "orange3.4.L.B",
    "orange3.7.R.B",
    "orange4.2.R.T",
    "orange4.4.L.T",
    "orange4.8.R.B",
}

# Each robot's own unloading station, and every unloading point it has a saved route to.
UNLOADING_LABELS = ["ul1", "ul2"]
UNLOADING_STATIONS = {
    "robot_a": {"label": "ul1", "points": {"ul1_1": "ul1", "ul1_2": "ul1", "ul2_2": "ul2"}},
    "robot_b": {"label": "ul2", "points": {"ul1_2": "ul1", "ul2_1": "ul2", "ul2_2": "ul2"}},
}

AUTO_TEST_ORDERS = {
    "robot_a": ["a{}".format(index) for index in range(1, 21)],
    "robot_b": ["a{}".format(index) for index in range(40, 20, -1)],
}

STEP_SETTINGS = ("planning_frame", "odom_topic", "result_topic", "cmd_vel_topic", "arm_command_topic", "gripper_command_topic")

PICK_STEPS = [
    ("Approach target fruit", "approach_orange.py"),
    ("Reach fruit with robotic arm", "moveit_target_reach.py"),
    ("Grasp fruit", "grasp_orange.py"),
]

USE_COLOR = sys.stdout.isatty()
COLOR_RESET = "\033[0m" if USE_COLOR else ""
COLOR_TEXT = "\033[96m" if USE_COLOR else ""
COLOR_DETAIL = "\033[92m" if USE_COLOR else ""

NAV_START_DELAY = 5.0
MAX_PICK_ATTEMPTS = 3
NAV_FINISH_DELAY = 3.0
UNLOAD_ACTION = "open"
INTER_ORANGE_DELAY = 2.0
STEP_SHUTDOWN_DELAY = 0.1
ORANGE_ACTION_TIMEOUT = 500.0
PROCESS_EXIT_GRACE_DELAY = 1.0

# Latest orange_action result per topic; None until the running step reports.
ORANGE_ACTION_RESULTS = {}


def info(text):
    print("{}[harvest_workflow] {}{}".format(COLOR_TEXT, text, COLOR_RESET))


def detail(text):
    print("{}  {}{}".format(COLOR_DETAIL, text, COLOR_RESET))


def prompt(text):
    return input("{}[harvest_workflow] {}{}".format(COLOR_TEXT, text, COLOR_RESET))


def prompt_yes_no(text, default):
    options = "[Y/n]" if default else "[y/N]"
    while True:
        entered = prompt("{} {}: ".format(text, options)).strip().lower()
        if not entered:
            return default
        if entered in ("y", "yes"):
            return True
        if entered in ("n", "no"):
            return False
        info("Please enter y or n.")


def destination_sort_key(destination_name):
    # Waypoints first, in number order, then the unloading stations.
    if destination_name in UNLOADING_LABELS:
        return (1, UNLOADING_LABELS.index(destination_name))
    return (0, int(destination_name[1:]))


def display_node_name(robot_key, node_name):
    return UNLOADING_STATIONS[robot_key]["points"].get(node_name, node_name)


def rospack_find(package_name):
    return subprocess.check_output(["rospack", "find", package_name], text=True).strip()


def path_directory(robot_key, node_name):
    folder = "wavepoint{}".format(node_name[1:]) if node_name.startswith("a") else node_name
    return os.path.join(rospack_find("harbot_navigation"), "paths", robot_key, folder)


def list_destinations(robot_key, current_node):
    prefix = current_node + "-"
    file_names = sorted(os.listdir(path_directory(robot_key, current_node)))
    return [file_name[len(prefix):-len(".json")] for file_name in file_names]


def resolve_destination_node(robot_key, current_node, selected_destination):
    if selected_destination not in UNLOADING_LABELS:
        return selected_destination

    for destination in list_destinations(robot_key, current_node):
        if UNLOADING_STATIONS[robot_key]["points"].get(destination) == selected_destination:
            return destination
    return selected_destination


def get_current_pose(robot_ns, timeout=5.0):
    message = rospy.wait_for_message(helpers.get_robot_topic(robot_ns, "odom_topic"), Odometry, timeout=timeout)
    q = message.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return message.pose.pose.position.x, message.pose.pose.position.y, yaw


def detect_current_node(robot_key, pose):
    best_name = None
    best_score = None
    for node_name, node_pose in WAYPOINTS[robot_key].items():
        distance = math.hypot(pose[0] - node_pose[0], pose[1] - node_pose[1])
        heading_error = abs(math.atan2(math.sin(pose[2] - node_pose[2]), math.cos(pose[2] - node_pose[2])))
        score = distance + 0.25 * heading_error
        if best_score is None or score < best_score:
            best_name = node_name
            best_score = score
    return best_name, best_score


def rosrun_command(package_name, script_name, private_params):
    command = ["rosrun", package_name, script_name]
    command.extend("_{}:={}".format(name, value) for name, value in private_params.items())
    return command


def start_process(command, quiet=False):
    output = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(command, start_new_session=True, stdout=output, stderr=output)


def shutdown_process(process_handle, timeout=5.0):
    if process_handle is None or process_handle.poll() is not None:
        return

    try:
        os.killpg(process_handle.pid, signal.SIGINT)
        process_handle.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process_handle.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def launch_navigation(robot_ns, route, gui_enabled):
    command = [
        "roslaunch",
        "harbot_navigation",
        "path_nav.launch",
        "route:={}".format(route),
        "robot_ns:={}".format(robot_ns),
        "gui:={}".format(str(gui_enabled).lower()),
        "odom_topic:={}".format(helpers.get_setting("odom_topic")),
        "status_topic:={}".format(helpers.get_setting("status_topic")),
        "cmd_vel_topic:={}".format(helpers.get_setting("cmd_vel_topic")),
        "proceed_flag_topic:={}".format(helpers.get_setting("proceed_flag_topic")),
    ]
    return start_process(command, quiet=True)


def launch_persistent_path_gui(robot_ns):
    command = [
        "rosrun",
        "harbot_navigation",
        "path_gui.py",
        "__ns:={}".format(robot_ns),
        "_local_path_topic:=path_nav/local_path",
        "_reverse_mode_topic:=path_nav/reverse_mode",
        "_desired_speed_topic:=path_nav/desired_speed",
        "_odom_topic:={}".format(helpers.get_setting("odom_topic")),
        "_status_topic:={}".format(helpers.get_setting("status_topic")),
        "_proceed_flag_topic:={}".format(helpers.get_setting("proceed_flag_topic")),
    ]
    return start_process(command, quiet=True)


def launch_camera_view(robot_key):
    rviz_config = os.path.join(rospack_find("harbot_simulation"), "rviz", "rbkairos_camera_views.rviz")
    command = ["rosrun", "rviz", "rviz", "-d", rviz_config, "__name:={}_camera_views_rviz".format(robot_key)]
    return start_process(command, quiet=True)


def send_proceed_flag(robot_ns):
    publisher = rospy.Publisher(helpers.get_robot_topic(robot_ns, "proceed_flag_topic"), Bool, queue_size=1, latch=True)
    publisher.publish(Bool(data=True))


def wait_for_navigation_finish(robot_ns, process_handle, timeout):
    status_topic = helpers.get_robot_topic(robot_ns, "status_topic")
    deadline = time.monotonic() + timeout
    last_status = None

    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False, "Timed out waiting for navigation after {:.1f}s.".format(timeout)

        if process_handle.poll() is not None:
            return False, "path_nav exited before reporting FINISHED."

        try:
            message = rospy.wait_for_message(status_topic, String, timeout=min(2.0, remaining))
        except rospy.ROSException:
            continue

        if message.data != last_status:
            rospy.loginfo("path_nav status: %s", message.data)
            last_status = message.data

        if message.data == "FINISHED":
            return True, "Navigation finished."

    return False, "ROS shutdown while waiting for navigation."


def navigate(robot_ns, route, gui_enabled, timeout, start_delay, finish_delay):
    process_handle = None
    try:
        process_handle = launch_navigation(robot_ns, route, gui_enabled)
        rospy.sleep(start_delay)
        send_proceed_flag(robot_ns)

        ok, message = wait_for_navigation_finish(robot_ns, process_handle, timeout)
        if ok:
            rospy.sleep(finish_delay)
    finally:
        shutdown_process(process_handle)

    return ok, message


def orange_action_callback(msg, topic):
    ORANGE_ACTION_RESULTS[topic] = msg.data


def wait_for_orange_action_result(topic, process_handle, step_label, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        result = ORANGE_ACTION_RESULTS[topic]
        if result is not None:
            return bool(result)

        # The script stopped: wait once for a result that may still be on its way.
        if process_handle.poll() is not None:
            rospy.sleep(PROCESS_EXIT_GRACE_DELAY)
            result = ORANGE_ACTION_RESULTS[topic]
            if result is None:
                raise RuntimeError("{} exited before publishing orange_action.".format(step_label))
            return bool(result)

        if time.monotonic() > deadline:
            raise RuntimeError("Timed out waiting for orange_action from {} after {:.1f} seconds.".format(step_label, timeout))
        rospy.sleep(0.1)

    return False


def run_orange_script_step(robot_ns, orange_name, step_label, script_name, timeout=ORANGE_ACTION_TIMEOUT, allow_failure_result=False):
    if timeout <= 0.0:
        raise RuntimeError("{} timed out before starting.".format(step_label))

    private_params = {"orange": orange_name, "robot_ns": robot_ns}
    private_params.update({setting_name: helpers.get_setting(setting_name) for setting_name in STEP_SETTINGS})

    # Subscribe once per topic, before the first step starts, so a fast result is not missed.
    topic = helpers.get_robot_topic(robot_ns, "result_topic")
    if topic not in ORANGE_ACTION_RESULTS:
        ORANGE_ACTION_RESULTS[topic] = None
        rospy.Subscriber(topic, Bool, orange_action_callback, callback_args=topic, queue_size=10)

    process_handle = None
    try:
        process_handle = start_process(rosrun_command("harbot_harvest", script_name, private_params))
        succeeded = wait_for_orange_action_result(topic, process_handle, step_label, timeout)
        result_time = time.monotonic()
        rospy.sleep(STEP_SHUTDOWN_DELAY)
    finally:
        shutdown_process(process_handle)
        # Clear the result, so the next step waits for its own.
        ORANGE_ACTION_RESULTS[topic] = None

    if not succeeded and not allow_failure_result:
        raise RuntimeError("{} reported orange_action=False.".format(step_label))

    return succeeded, result_time


def run_container_action(robot_ns, action, timeout):
    process_name = "container_control.py ({})".format(action)
    process_handle = None
    try:
        command = rosrun_command("harbot_harvest", "container_control.py", {"robot_ns": robot_ns, "action": action})
        process_handle = start_process(command)
        return_code = process_handle.wait(timeout)
    except subprocess.TimeoutExpired:
        return False, "Timed out waiting for {} after {:.1f}s.".format(process_name, timeout)
    finally:
        shutdown_process(process_handle)

    if return_code != 0:
        return False, "{} exited with code {}.".format(process_name, return_code)
    return True, "{} completed.".format(process_name)


def execute_fruit_sequence(robot_ns, selected_fruits):
    steps = PICK_STEPS + [("Unload fruit from gripper", "unload_orange.py")]
    for orange_name in selected_fruits:
        if orange_name in UNRIPE_ORANGES:
            info("{} is unripe. Skipping harvest.".format(orange_name))
            continue

        harvested = False
        for attempt in range(1, MAX_PICK_ATTEMPTS + 1):
            info("Harvesting {} (attempt {}/{}).".format(orange_name, attempt, MAX_PICK_ATTEMPTS))
            try:
                for index, (step_label, script_name) in enumerate(steps, start=1):
                    label = "Step {}/{}: {}".format(index, len(steps), step_label)
                    info(label)
                    run_orange_script_step(robot_ns, orange_name, label, script_name)
                harvested = True
                break
            except (OSError, RuntimeError, rospy.ROSException) as exc:
                info("Harvesting workflow failed for {}: {}".format(orange_name, exc))

            if attempt < MAX_PICK_ATTEMPTS:
                rospy.sleep(INTER_ORANGE_DELAY)

        if not harvested:
            info("Giving up on {} after {} attempts.".format(orange_name, MAX_PICK_ATTEMPTS))
            return False

        info("Completed harvesting workflow for {}.".format(orange_name))
        rospy.sleep(INTER_ORANGE_DELAY)

    return True


def prompt_destination(destinations):
    info("Available destinations:")
    for index, destination_name in enumerate(destinations, start=1):
        detail("{}. {}".format(index, destination_name))
    detail("0. Exit")

    while True:
        selected = prompt("Select destination number: ").strip()
        if selected == "0":
            return None
        if selected.isdigit() and 1 <= int(selected) <= len(destinations):
            return destinations[int(selected) - 1]
        info("Please enter a valid option number.")


def prompt_fruit_selection(location_name):
    if location_name not in FRUIT_OPTIONS:
        return []

    fruits = FRUIT_OPTIONS[location_name]
    info("Fruit options at {}:".format(location_name))
    for index, fruit_name in enumerate(fruits, start=1):
        detail("{}. {}".format(index, fruit_name))

    info("Enter fruit numbers separated by spaces or commas, or press Enter to skip.")
    while True:
        tokens = prompt("Fruit selection: ").replace(",", " ").split()
        if not tokens:
            return []
        if all(token.isdigit() and 1 <= int(token) <= len(fruits) for token in tokens):
            chosen = [fruits[int(token) - 1] for token in tokens]
            info("Selected fruits: {}".format(", ".join(chosen)))
            return chosen
        info("Please enter numbers from the list.")


def navigate_to_node(robot_ns, robot_key, start_node, destination_node):
    route = "{}-{}".format(start_node, destination_node)
    info("Launching path_nav with route={} ...".format(route))

    ok, message = navigate(robot_ns, route, False, float("inf"), NAV_START_DELAY, NAV_FINISH_DELAY)
    if not ok:
        raise rospy.ROSException(message)

    reached_node, reached_score = detect_current_node(robot_key, get_current_pose(robot_ns))
    info("Robot is now nearest to {} (match score {:.3f}).".format(display_node_name(robot_key, reached_node), reached_score))
    return reached_node


def run_full_automatic_test(robot_ns, robot_key, current_node):
    waypoint_order = AUTO_TEST_ORDERS[robot_key]
    if current_node not in waypoint_order:
        info("Full automatic test can only start from a test waypoint, not from {}.".format(display_node_name(robot_key, current_node)))
        return False

    info("Starting full automatic test from {}.".format(current_node))
    if not execute_fruit_sequence(robot_ns, FRUIT_OPTIONS[current_node]):
        return False

    active_node = current_node
    for next_waypoint in waypoint_order[waypoint_order.index(current_node) + 1:]:
        try:
            active_node = navigate_to_node(robot_ns, robot_key, active_node, next_waypoint)
        except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
            info("Automatic navigation failed while going to {}: {}".format(next_waypoint, exc))
            return False

        if active_node != next_waypoint:
            info("Expected to reach {}, but detected {} instead.".format(next_waypoint, display_node_name(robot_key, active_node)))
            return False

        if not execute_fruit_sequence(robot_ns, FRUIT_OPTIONS[active_node]):
            return False

    unloading_label = UNLOADING_STATIONS[robot_key]["label"]
    destination_node = resolve_destination_node(robot_key, active_node, unloading_label)
    info("Automatic test finished waypoint collection. Going to {}.".format(unloading_label))
    try:
        reached_node = navigate_to_node(robot_ns, robot_key, active_node, destination_node)
    except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
        info("Automatic navigation to {} failed: {}".format(unloading_label, exc))
        return False

    if reached_node not in UNLOADING_STATIONS[robot_key]["points"]:
        info("Expected to reach {}, but detected {} instead.".format(unloading_label, display_node_name(robot_key, reached_node)))
        return False

    info("Running container unload action: {}.".format(UNLOAD_ACTION))
    ok, message = run_container_action(robot_ns, UNLOAD_ACTION, float("inf"))
    if not ok:
        info(message)
        return False

    info("Full automatic test completed.")
    return True


def main():
    rospy.init_node("harvest_workflow", anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ns")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--auto-test", action="store_true")
    parser.add_argument("--camera-view", action="store_true")
    arguments = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    try:
        robot_ns = helpers.normalize_robot_namespace(arguments.robot_ns or prompt("Robot [a/b]: ").strip() or "a")
        robot_key = helpers.robot_key(robot_ns)
    except ValueError as exc:
        info(exc)
        return 1

    gui_requested = arguments.gui or prompt_yes_no("Open the path GUI?", default=False)
    camera_view_requested = arguments.camera_view or prompt_yes_no("Open third-person camera view?", default=False)
    auto_test_requested = arguments.auto_test or prompt_yes_no("Run full automatic test from the current waypoint?", default=False)
    path_gui_process = None
    camera_view_process = None

    try:
        if gui_requested:
            path_gui_process = launch_persistent_path_gui(robot_ns)
            info("Persistent path GUI started for {}.".format(robot_ns))
            rospy.sleep(1.0)

        if camera_view_requested:
            camera_view_process = launch_camera_view(robot_key)
            info("Third-person camera view started.")

        info("Listening to {}...".format(robot_ns))

        while not rospy.is_shutdown():
            try:
                current_pose = get_current_pose(robot_ns)
            except rospy.ROSException as exc:
                info("Failed to read odometry from {}: {}".format(robot_ns, exc))
                return 1

            current_node, score = detect_current_node(robot_key, current_pose)
            current_location_name = display_node_name(robot_key, current_node)
            info("Current pose is nearest to {} (match score {:.3f}).".format(current_location_name, score))

            if auto_test_requested:
                if run_full_automatic_test(robot_ns, robot_key, current_node):
                    return 0
                info("Automatic test did not complete.")
                return 1

            if not prompt_yes_no("Current location is {}. Navigate to another destination?".format(current_location_name), default=True):
                execute_fruit_sequence(robot_ns, prompt_fruit_selection(current_node))
                if not prompt_yes_no("Stay in the script and choose another destination?", default=True):
                    info("Exiting interactive waypoint navigator.")
                    return 0
                continue

            shown_names = {display_node_name(robot_key, destination) for destination in list_destinations(robot_key, current_node)}
            destinations = sorted(shown_names, key=destination_sort_key)

            destination_waypoint = prompt_destination(destinations)
            if destination_waypoint is None:
                info("Exiting interactive waypoint navigator.")
                return 0

            destination_node = resolve_destination_node(robot_key, current_node, destination_waypoint)
            try:
                reached_node = navigate_to_node(robot_ns, robot_key, current_node, destination_node)
            except (subprocess.CalledProcessError, OSError, rospy.ROSException) as exc:
                info("Navigation failed: {}".format(exc))
                return 1
            execute_fruit_sequence(robot_ns, prompt_fruit_selection(reached_node))

        return 0
    finally:
        shutdown_process(path_gui_process)
        shutdown_process(camera_view_process)


if __name__ == "__main__":
    sys.exit(main())
