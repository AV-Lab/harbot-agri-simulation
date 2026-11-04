#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Offer the PROST sim_action_server service for one robot.
# - Decode the float64[6] action input into a waypoint, station, or orange.
# - navigate: run path_nav from the current node to the requested destination.
# - grasp_fruit: approach, reach, and grasp an orange available at this node.
# - load_to_bin: drop the held orange into the robot's container.
# - unload: open and close the container bottom.
# - Report the MoveIt planning time and the time from planning to grasp.
# ******************************************************************************


import os
import time
import rospy
import threading
from std_msgs.msg import Float64
from harbot_harvest import helpers, workflow
from rbkairos_etf_services.srv import ActionServer, ActionServerResponse


class ProstSimActionServer:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))
        self.robot_key = helpers.robot_key(self.robot_ns)

        self.gui_enabled = False
        self.nav_start_delay = 1.1
        self.default_timeout = 120.0
        self.nav_finish_delay = 1.1
        self.service_name = self.robot_ns + "/sim_action_server"

        self.held_orange = None
        self.lock = threading.Lock()
        self.planning_time_sec = 0.0
        self.latest_planning_time = None
        self.latest_planning_done_time = None
        self.post_planning_grasp_time_sec = 0.0

        self.service = rospy.Service(self.service_name, ActionServer, self.handle_action)

        rospy.Subscriber(self.robot_ns + "/moveit_planning_time_sec", Float64, self.planning_time_callback, queue_size=1)
        rospy.Subscriber(self.robot_ns + "/moveit_planning_done_time", Float64, self.planning_done_time_callback, queue_size=1)

        rospy.loginfo("PROST simulation action server ready on %s for robot_ns=%s", self.service_name, self.robot_ns)

    def planning_time_callback(self, msg):
        self.latest_planning_time = msg.data

    def planning_done_time_callback(self, msg):
        self.latest_planning_done_time = msg.data

    def handle_action(self, request):
        if not self.lock.acquire(False):
            return ActionServerResponse(False, "Server is already executing an action.", 0.0, 0.0)

        actions = {
            "unload": self.action_unload,
            "navigate": self.action_navigate,
            "grasp_fruit": self.action_grasp_fruit,
            "load_to_bin": self.action_load_to_bin,
        }

        try:
            self.planning_time_sec = 0.0
            self.post_planning_grasp_time_sec = 0.0
            action_id = request.action_id.strip()
            arr = list(request.input)
            timeout = float(request.timeout) if request.timeout > 0.0 else self.default_timeout

            if len(arr) != 6 or not all(float(value).is_integer() for value in arr):
                return ActionServerResponse(False, "Input must be 6 whole numbers, got {}.".format(arr), 0.0, 0.0)
            arr = [int(value) for value in arr]

            if action_id in actions:
                ok, message = actions[action_id](arr, timeout)
            elif action_id in ("NOOP", "wait"):
                ok, message = True, "{}: success.".format(action_id)
            else:
                ok, message = False, "Unknown action_id '{}'. Supported: navigate, grasp_fruit, load_to_bin, unload, NOOP.".format(action_id)

            rospy.loginfo("Sim action result: success=%s message=%s", ok, message)
            return ActionServerResponse(ok, message, self.planning_time_sec, self.post_planning_grasp_time_sec)
        except Exception as exc:
            rospy.logerr("Sim action failed: %s", exc)
            return ActionServerResponse(False, str(exc), 0.0, 0.0)
        finally:
            self.lock.release()

    def action_navigate(self, arr, timeout):
        current_node, _ = self.current_node()
        destination = self.resolve_navigation_destination(current_node, self.decode_navigation_target(arr))
        rospy.loginfo("Current node is %s, target is %s.", self.display_name(current_node), self.display_name(destination))

        if current_node == destination:
            return True, "Already at {}.".format(self.display_name(destination))

        route = "{}-{}".format(current_node, destination)
        path_file = os.path.join(workflow.path_directory(self.robot_key, current_node), route + ".json")
        if not os.path.isfile(path_file):
            return False, "No saved path file from {} to {}: {}".format(current_node, destination, path_file)

        rospy.loginfo("Launching path_nav with route=%s", route)
        ok, message = workflow.navigate(self.robot_ns, route, self.gui_enabled, timeout, self.nav_start_delay, self.nav_finish_delay)
        if not ok:
            return False, message

        reached_node, _ = self.current_node()
        if reached_node != destination:
            return False, "Expected {}, but nearest node is {}.".format(self.display_name(destination), self.display_name(reached_node))

        return True, "Navigation reached {}.".format(self.display_name(destination))

    def action_grasp_fruit(self, arr, timeout):
        orange_name = self.decode_orange(arr)
        current_node, _ = self.current_node()
        fruit_options = workflow.FRUIT_OPTIONS.get(current_node, [])
        if orange_name not in fruit_options:
            options = ", ".join(fruit_options) if fruit_options else "none"
            raise ValueError("{} is not available at current node {}. Options: {}".format(orange_name, self.display_name(current_node), options))
        if orange_name in workflow.UNRIPE_ORANGES:
            raise ValueError("{} is configured as unripe.".format(orange_name))

        start_time = time.time()
        planning_done_time = 0.0
        for index, (step_label, script_name) in enumerate(workflow.PICK_STEPS, start=1):
            label = "Step {}/{}: {}".format(index, len(workflow.PICK_STEPS), step_label)

            # MoveIt reach is retried until it succeeds or the action times out.
            while not rospy.is_shutdown():
                succeeded, result_time = self.run_step(orange_name, label, script_name, start_time, timeout)
                if script_name != "moveit_target_reach.py":
                    break

                rospy.sleep(0.5)
                self.planning_time_sec += self.latest_planning_time or 0.0
                planning_done_time = self.latest_planning_done_time or planning_done_time
                # Clear the planning times, so the next reach attempt reports its own.
                self.latest_planning_time = None
                self.latest_planning_done_time = None
                if succeeded:
                    break
                rospy.sleep(0.1)

            if script_name == "grasp_orange.py":
                if planning_done_time > 0.0:
                    self.post_planning_grasp_time_sec = result_time - planning_done_time
                if not succeeded:
                    self.held_orange = None
                    return False, "Grasp failed for {}.".format(orange_name)

        self.held_orange = orange_name
        return True, "Grasped {}.".format(orange_name)

    def action_load_to_bin(self, arr, timeout):
        orange_name = self.decode_orange(arr)
        if self.held_orange != orange_name:
            return False, "Requested load_to_bin for {}, but the robot is holding {}.".format(orange_name, self.held_orange or "nothing")

        self.run_step(orange_name, "Load fruit to bin", "unload_orange.py", time.time(), timeout)
        self.held_orange = None

        return True, "Loaded {} to bin.".format(orange_name)

    def action_unload(self, _arr, timeout):
        start_time = time.time()
        for container_action in ("open", "close"):
            remaining = max(0.0, timeout - (time.time() - start_time))
            if remaining <= 0.0:
                return False, "Timed out before container {} action.".format(container_action)

            ok, message = workflow.run_container_action(self.robot_ns, container_action, remaining)
            if not ok:
                return False, message

        return True, "Container unload action completed: opened and closed."

    def run_step(self, orange_name, label, script_name, start_time, timeout):
        rospy.loginfo("%s for %s", label, orange_name)
        step_timeout = max(0.0, timeout - (time.time() - start_time))
        allow_failure = script_name in ("moveit_target_reach.py", "grasp_orange.py")
        return workflow.run_orange_script_step(self.robot_ns, orange_name, label, script_name, step_timeout, allow_failure)

    def decode_navigation_target(self, arr):
        # [5, 0, 0, 0, 0, 0] -> a5
        # [0, 0, 0, 0, 1, 0] -> ul1
        # [0, 0, 0, 0, 0, 1] -> ul2
        first_four = arr[:4]
        unload_flags = arr[4:]

        if first_four == [0, 0, 0, 0]:
            if unload_flags == [1, 0]:
                return "ul1"
            if unload_flags == [0, 1]:
                return "ul2"
            if unload_flags != [0, 0]:
                raise ValueError("Invalid unloading station flags {}. Use [0,0,0,0,1,0] for ul1 or [0,0,0,0,0,1] for ul2.".format(unload_flags))

        waypoint_name = "a{}".format(first_four[0])
        if waypoint_name not in workflow.WAYPOINTS[self.robot_key]:
            raise ValueError("{} is not a waypoint of {}.".format(waypoint_name, self.robot_key))

        return waypoint_name

    def resolve_navigation_destination(self, current_node, requested_destination):
        if requested_destination not in workflow.UNLOADING_LABELS:
            return requested_destination

        if self.display_name(current_node) == requested_destination:
            return current_node

        destination = workflow.resolve_destination_node(self.robot_key, current_node, requested_destination)
        if destination == requested_destination:
            raise ValueError("No saved path from {} to {}.".format(self.display_name(current_node), requested_destination))

        return destination

    def decode_orange(self, arr):
        # [row, tree, side, height, 0, 0]
        # [1, 1, 0, 1, 0, 0] -> orange1.1.L.T
        # [2, 7, 1, 0, 0, 0] -> orange2.7.R.B
        row, tree, side_code, height_code = arr[:4]

        # parse_orange() rejects rows, trees, sides and heights that don't exist.
        orange_name = "orange{}.{}.{}.{}".format(row, tree, {0: "L", 1: "R"}.get(side_code, side_code), {0: "B", 1: "T"}.get(height_code, height_code))
        helpers.parse_orange(orange_name)
        return orange_name

    def current_node(self):
        return workflow.detect_current_node(self.robot_key, workflow.get_current_pose(self.robot_ns, timeout=5.0))

    def display_name(self, node_name):
        return workflow.display_node_name(self.robot_key, node_name)


def main():
    rospy.init_node("prost_sim_action_server", anonymous=True)
    ProstSimActionServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
