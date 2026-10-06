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
import math
import time
import rospy
import threading
from std_msgs.msg import Float64
from harbot_harvest import helpers, ros_utils, workflow
from rbkairos_etf_services.srv import ActionServer, ActionServerResponse


class ProstSimActionServer:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))
        self.robot_key = helpers.robot_key(self.robot_ns)

        self.gui_enabled = bool(rospy.get_param("~gui", False))
        self.nav_start_delay = float(rospy.get_param("~nav_start_delay", 1.1))
        self.default_timeout = float(rospy.get_param("~default_timeout", 120.0))
        self.nav_finish_delay = float(rospy.get_param("~nav_finish_delay", 1.1))
        self.service_name = rospy.get_param("~service_name", self.robot_ns + "/sim_action_server")

        self.held_orange = None
        self.lock = threading.Lock()
        self.planning_time_sec = 0.0
        self.post_planning_grasp_time_sec = 0.0
        self.planning_time = ros_utils.LatestMessage(self.robot_ns + "/moveit_planning_time_sec", Float64)
        self.planning_done_time = ros_utils.LatestMessage(self.robot_ns + "/moveit_planning_done_time", Float64)

        self.service = rospy.Service(self.service_name, ActionServer, self.handle_action)

        rospy.loginfo("PROST simulation action server ready on %s for robot_ns=%s", self.service_name, self.robot_ns)

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

            if len(arr) != 6:
                return ActionServerResponse(False, "Input must be float64[6].", 0.0, 0.0)

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

        route = workflow.build_path_name(current_node, destination)
        path_file = workflow.route_file(self.robot_key, current_node, destination)
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
        self.assert_orange_available_here(orange_name)

        start_time = time.time()
        planning_done_time = 0.0
        for index, (step_label, script_name) in enumerate(workflow.PICK_STEPS, start=1):
            label = "Step {}/{}: {}".format(index, len(workflow.PICK_STEPS), step_label)

            # MoveIt reach is retried until it succeeds or the action times out.
            while not rospy.is_shutdown():
                succeeded, step_start, result_time = self.run_step(orange_name, label, script_name, start_time, timeout)
                if script_name != "moveit_target_reach.py":
                    break

                self.planning_time_sec += self.planning_time.wait_since(step_start)
                planning_done_time = self.planning_done_time.wait_since(step_start) or planning_done_time
                if succeeded:
                    break
                rospy.sleep(0.1)

            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown while executing {}.".format(label))

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
        if self.held_orange is not None and self.held_orange != orange_name:
            return False, "Requested load_to_bin for {}, but held orange is {}.".format(orange_name, self.held_orange)

        self.run_step(orange_name, "Load fruit to bin", "unload_orange.py", time.time(), timeout)
        if self.held_orange == orange_name:
            self.held_orange = None

        return True, "Loaded {} to bin.".format(orange_name)

    def action_unload(self, _arr, timeout):
        start_time = time.time()
        for container_action in ("open", "close"):
            remaining = self.remaining_timeout(start_time, timeout)
            if remaining <= 0.0:
                return False, "Timed out before container {} action.".format(container_action)

            ok, message = workflow.run_container_action(self.robot_ns, container_action, remaining)
            if not ok:
                return False, message

        return True, "Container unload action completed: opened and closed."

    def run_step(self, orange_name, label, script_name, start_time, timeout):
        rospy.loginfo("%s for %s", label, orange_name)
        step_timeout = self.remaining_timeout(start_time, timeout)
        allow_failure = script_name in ("moveit_target_reach.py", "grasp_orange.py")
        return workflow.run_orange_script_step(self.robot_ns, orange_name, label, script_name, step_timeout, allow_failure)

    def decode_navigation_target(self, arr):
        # [waypoint_id, 0, 0, 0, 0, 0] -> a<waypoint_id>
        # [0, 0, 0, 0, 1, 0] -> ul1
        # [0, 0, 0, 0, 0, 1] -> ul2
        first_four = [self.as_int(value, "navigation field {}".format(index + 1)) for index, value in enumerate(arr[:4])]
        unload_flags = [self.as_int(arr[4], "ul1 flag"), self.as_int(arr[5], "ul2 flag")]

        if first_four == [0, 0, 0, 0]:
            if unload_flags == [1, 0]:
                return "ul1"
            if unload_flags == [0, 1]:
                return "ul2"
            if unload_flags != [0, 0]:
                raise ValueError("Invalid unloading station flags {}. Use [0,0,0,0,1,0] for ul1 or [0,0,0,0,0,1] for ul2.".format(unload_flags))

        waypoint_name = "a{}".format(self.as_int(arr[0], "waypoint id"))
        waypoints = workflow.WAYPOINTS[self.robot_key]
        if waypoint_name not in waypoints:
            valid_names = ", ".join(sorted(waypoints, key=workflow.waypoint_sort_key))
            raise ValueError("{} is not valid for {}. Valid waypoints are {}.".format(waypoint_name, self.robot_key, valid_names))

        return waypoint_name

    def resolve_navigation_destination(self, current_node, requested_destination):
        if requested_destination not in workflow.UNLOADING_LABELS:
            return requested_destination

        if workflow.is_unloading_point(self.robot_key, current_node) and self.display_name(current_node) == requested_destination:
            return current_node

        destination = workflow.resolve_destination_node(self.robot_key, current_node, requested_destination)
        if destination == requested_destination:
            raise ValueError("No saved path from {} to {}.".format(self.display_name(current_node), requested_destination))

        return destination

    def decode_orange(self, arr):
        # [row, tree, side, height, 0, 0], side: 0 = L, 1 = R, height: 0 = B, 1 = T
        # [1, 1, 0, 1, 0, 0] -> orange1.1.L.T
        # [2, 7, 1, 0, 0, 0] -> orange2.7.R.B
        row = self.as_int(arr[0], "orange row")
        tree = self.as_int(arr[1], "orange tree")
        side_code = self.as_int(arr[2], "orange side")
        height_code = self.as_int(arr[3], "orange height")

        if not 1 <= row <= 4:
            raise ValueError("Orange row must be 1..4, got {}.".format(row))
        if not 1 <= tree <= 8:
            raise ValueError("Orange tree must be 1..8, got {}.".format(tree))
        if side_code not in (0, 1):
            raise ValueError("Orange side must be 0 for L or 1 for R, got {}.".format(side_code))
        if height_code not in (0, 1):
            raise ValueError("Orange height must be 0 for B or 1 for T, got {}.".format(height_code))

        return helpers.format_orange(row, tree, "LR"[side_code], "BT"[height_code])

    def assert_orange_available_here(self, orange_name):
        current_node, _ = self.current_node()
        fruit_options = workflow.FRUIT_OPTIONS.get(current_node, [])

        if orange_name not in fruit_options:
            options = ", ".join(fruit_options) if fruit_options else "none"
            raise ValueError("{} is not available at current node {}. Options: {}".format(orange_name, self.display_name(current_node), options))
        if orange_name in workflow.UNRIPE_ORANGES:
            raise ValueError("{} is configured as unripe.".format(orange_name))

    def current_node(self):
        return workflow.detect_current_node(self.robot_key, workflow.get_current_pose(self.robot_ns, timeout=5.0))

    def display_name(self, node_name):
        return workflow.display_node_name(self.robot_key, node_name)

    def remaining_timeout(self, start_time, timeout):
        return max(0.0, timeout - (time.time() - start_time))

    def as_int(self, value, label):
        number = float(value)
        rounded = int(round(number))
        if math.fabs(number - rounded) > 1e-6:
            raise ValueError("{} must be an integer-like value, got {}.".format(label, value))
        return rounded


def main():
    rospy.init_node("prost_sim_action_server", anonymous=True)
    ProstSimActionServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
