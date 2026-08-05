#!/usr/bin/env python3

import math
import os
import threading
import time

import rospy
from std_msgs.msg import String

from rbkairos_etf_services.srv import ActionServer, ActionServerResponse

import harvest_workflow as workflow


class ProstSimActionServer(object):
    def __init__(self):
        self.robot_ns = workflow.normalize_namespace(rospy.get_param("~robot_ns", "/robot"))
        self.robot_key = workflow.namespace_to_path_key(self.robot_ns)

        default_service_name = "{}/sim_action_server".format(self.robot_ns)
        self.service_name = rospy.get_param("~service_name", default_service_name)

        self.gui_enabled = bool(rospy.get_param("~gui", False))
        self.arm_type = rospy.get_param("~arm", workflow.PICK_ARM_TYPE)
        self.default_timeout = float(rospy.get_param("~default_timeout", 120.0))
        self.nav_start_delay = float(rospy.get_param("~nav_start_delay", 1.1))
        self.nav_finish_delay = float(rospy.get_param("~nav_finish_delay", 1.1))
        self.container_action = rospy.get_param("~container_action", "open")

        self.held_orange = None
        self.lock = threading.Lock()

        self.service = rospy.Service(self.service_name, ActionServer, self.handle_action)
        rospy.loginfo(
            "PROST simulation action server ready on %s for robot_ns=%s",
            self.service_name,
            self.robot_ns,
        )

    def handle_action(self, request):
        moveit_planning_time_sec = 0.0
        post_moveit_planning_grasp_time_sec = 0.0
        if not self.lock.acquire(False):
            return ActionServerResponse(False, "Server is already executing an action.", 0.0, 0.0)

        try:
            action_id = request.action_id.strip()
            arr = list(request.input)
            timeout = float(request.timeout) if request.timeout > 0.0 else self.default_timeout

            if len(arr) != 6:
                return ActionServerResponse(False, "Input must be float64[6].", 0.0, 0.0)

            # rospy.loginfo(
            #     "Sim action request: action_id=%s input=%s",
            #     action_id,
            #     arr,
            # )

            if action_id == "navigate":
                ok, message = self.action_navigate(arr, timeout)
            elif action_id == "grasp_fruit":
                ok, message, moveit_planning_time_sec, post_moveit_planning_grasp_time_sec = self.action_grasp_fruit(arr, timeout)
            elif action_id == "load_to_bin":
                ok, message = self.action_load_to_bin(arr, timeout)
            elif action_id == "unload":
                ok, message = self.action_unload(timeout)
            elif action_id in ("NOOP", "wait"):
                ok, message = True, "{}: success.".format(action_id)
            else:
                ok = False
                message = "Unknown action_id '{}'. Supported: navigate, grasp_fruit, load_to_bin, unload, NOOP.".format(
                    action_id
                )

            rospy.loginfo("Sim action result: success=%s message=%s", ok, message)
            return ActionServerResponse(ok, message, moveit_planning_time_sec, post_moveit_planning_grasp_time_sec)

        except Exception as exc:
            rospy.logerr("Sim action failed: %s", exc)
            return ActionServerResponse(False, str(exc), 0.0, 0.0)
        finally:
            self.lock.release()

    def action_navigate(self, arr, timeout):
        current_node, _ = self.current_node()
        requested_destination = self.decode_navigation_target(arr)
        destination = self.resolve_navigation_destination(current_node, requested_destination)

        rospy.loginfo(
            "Current node is %s, target is %s.",
            workflow.display_node_name(self.robot_key, current_node),
            workflow.display_node_name(self.robot_key, destination),
        )

        if current_node == destination:
            return True, "Already at {}.".format(
                workflow.display_node_name(self.robot_key, destination)
            )

        path_name = workflow.build_path_name(self.robot_key, current_node, destination)
        path_file = os.path.join(workflow.rospack_find("rbkairos_path_nav"), "paths", path_name)
        if not os.path.isfile(path_file):
            return False, "No saved path file from {} to {}: {}".format(
                current_node,
                destination,
                path_file,
            )

        process_handle = None
        try:
            rospy.loginfo("Launching path_nav with path_name=%s", path_name)
            process_handle = workflow.launch_navigation(self.robot_ns, path_name, self.gui_enabled)
            rospy.sleep(self.nav_start_delay)
            workflow.send_proceed_flag(self.robot_ns)

            ok, message = self.wait_for_navigation_finish(process_handle, timeout)
            if not ok:
                return False, message

            rospy.sleep(self.nav_finish_delay)
        finally:
            workflow.shutdown_process(process_handle)

        reached_node, _ = self.current_node()
        if reached_node != destination:
            return False, "Expected {}, but nearest node is {}.".format(
                workflow.display_node_name(self.robot_key, destination),
                workflow.display_node_name(self.robot_key, reached_node),
            )

        return True, "Navigation reached {}.".format(
            workflow.display_node_name(self.robot_key, destination)
        )

    def action_grasp_fruit(self, arr, timeout):
        orange_name = self.decode_orange(arr)
        self.assert_orange_available_here(orange_name)

        start_time = time.time()
        moveit_planning_time_sec = 0.0
        moveit_planning_done_time = 0.0
        post_moveit_planning_grasp_time_sec = 0.0
        steps = [
            (
                "Step 1/3: Approach target fruit",
                "approach_orange.py",
                {"robot_ns": self.robot_ns},
            ),
            (
                "Step 2/3: Reach fruit with robotic arm",
                "moveit_target_reach.py",
                {"robot_ns": self.robot_ns, "arm": self.arm_type},
            ),
            (
                "Step 3/3: Grasp fruit",
                "grasp_orange.py",
                {"robot_ns": self.robot_ns, "arm": self.arm_type},
            ),
        ]

        for label, script_name, params in steps:
            step_succeeded = False
            step_result_time = 0.0
            moveit_reach_attempt = 0
            while not rospy.is_shutdown():
                step_succeeded, step_moveit_planning_time_sec, step_moveit_planning_done_time, step_result_time = self.run_orange_script_step(
                    orange_name,
                    label,
                    script_name,
                    params,
                    self.remaining_timeout(start_time, timeout),
                    allow_failure_result=script_name in ("moveit_target_reach.py", "grasp_orange.py"),
                )
                moveit_planning_time_sec += step_moveit_planning_time_sec
                if step_moveit_planning_done_time > 0.0:
                    moveit_planning_done_time = step_moveit_planning_done_time
                if script_name != "moveit_target_reach.py" or step_succeeded:
                    break

                moveit_reach_attempt += 1
                # rospy.logwarn(
                #     "MoveIt reach failed for %s after internal planning attempts; retrying reach step (retry %d).",
                #     orange_name,
                #     moveit_reach_attempt,
                # )
                rospy.sleep(0.1)

            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown while executing {}.".format(label))
            if script_name == "grasp_orange.py" and moveit_planning_done_time > 0.0:
                post_moveit_planning_grasp_time_sec = step_result_time - moveit_planning_done_time
                # rospy.loginfo(
                #     "\033[91m[POST MOVEIT TIMER STOP] %s elapsed %.3f sec\033[0m",
                #     orange_name,
                #     post_moveit_planning_grasp_time_sec,
                # )
            if script_name == "grasp_orange.py" and not step_succeeded:
                self.held_orange = None
                return False, "Grasp failed for {}.".format(orange_name), moveit_planning_time_sec, post_moveit_planning_grasp_time_sec

        self.held_orange = orange_name
        return True, "Grasped {}.".format(orange_name), moveit_planning_time_sec, post_moveit_planning_grasp_time_sec

    def action_load_to_bin(self, arr, timeout):
        orange_name = self.decode_orange(arr)
        if self.held_orange is not None and self.held_orange != orange_name:
            return False, "Requested load_to_bin for {}, but held orange is {}.".format(
                orange_name,
                self.held_orange,
            )

        self.run_orange_script_step(
            orange_name,
            "Load fruit to bin",
            "unload_orange.py",
            {"robot_ns": self.robot_ns},
            timeout,
        )

        if self.held_orange == orange_name:
            self.held_orange = None

        return True, "Loaded {} to bin.".format(orange_name)

    def action_unload(self, timeout):
        start_time = time.time()

        for container_action in ("open", "close"):
            remaining = self.remaining_timeout(start_time, timeout)
            if remaining <= 0.0:
                return False, "Timed out before container {} action.".format(container_action)

            process_handle = None
            try:
                process_handle = workflow.launch_ros_script(
                    "container_control.py",
                    {
                        "robot_ns": self.robot_ns,
                        "action": container_action,
                    },
                )
                ok, message = self.wait_for_process(
                    process_handle,
                    remaining,
                    "container_control.py ({})".format(container_action),
                )
                if not ok:
                    return False, message
            finally:
                workflow.shutdown_process(process_handle)

        return True, "Container unload action completed: opened and closed."

    def decode_navigation_target(self, arr):
        # Navigation encoding:
        # [waypoint_id, 0, 0, 0, 0, 0] -> a<waypoint_id>
        # [0, 0, 0, 0, 1, 0] -> ul1
        # [0, 0, 0, 0, 0, 1] -> ul2
        first_four = [
            self.as_int(value, "navigation field {}".format(index + 1))
            for index, value in enumerate(arr[:4])
        ]
        unload_flags = [self.as_int(arr[4], "ul1 flag"), self.as_int(arr[5], "ul2 flag")]

        if first_four == [0, 0, 0, 0]:
            if unload_flags == [1, 0]:
                return self.validate_unloading_label("ul1")
            if unload_flags == [0, 1]:
                return self.validate_unloading_label("ul2")
            if unload_flags != [0, 0]:
                raise ValueError(
                    "Invalid unloading station flags {}. Use [0,0,0,0,1,0] for ul1 or [0,0,0,0,0,1] for ul2.".format(
                        unload_flags
                    )
                )

        waypoint_id = self.as_int(arr[0], "waypoint id")
        waypoint_name = "a{}".format(waypoint_id)

        if waypoint_name not in workflow.WAYPOINTS[self.robot_key]:
            raise ValueError(
                "{} is not valid for {}. Valid waypoints are {}.".format(
                    waypoint_name,
                    self.robot_key,
                    ", ".join(sorted(workflow.WAYPOINTS[self.robot_key].keys(), key=workflow.waypoint_sort_key)),
                )
            )

        return waypoint_name

    def validate_unloading_label(self, unloading_label):
        valid_labels = workflow.unloading_station_labels(self.robot_key)
        if unloading_label not in valid_labels:
            raise ValueError(
                "{} is not valid for {}. This robot uses {}.".format(
                    unloading_label,
                    self.robot_key,
                    ", ".join(valid_labels),
                )
            )
        return unloading_label

    def resolve_navigation_destination(self, current_node, requested_destination):
        if requested_destination not in ("ul1", "ul2"):
            return requested_destination

        if (
            workflow.is_unloading_point(self.robot_key, current_node)
            and workflow.display_node_name(self.robot_key, current_node) == requested_destination
        ):
            return current_node

        destination = workflow.resolve_destination_node(
            self.robot_key,
            current_node,
            requested_destination,
        )
        if destination == requested_destination:
            raise ValueError(
                "No saved path from {} to {}.".format(
                    workflow.display_node_name(self.robot_key, current_node),
                    requested_destination,
                )
            )
        return destination

    def decode_orange(self, arr):
        # Orange encoding in actions.json:
        # [row, tree, side, height, 0, 0]
        # side:   0 = L, 1 = R
        # height: 0 = B, 1 = T
        #
        # Examples:
        # [1, 1, 0, 1, 0, 0] -> orange1.1.L.T
        # [2, 7, 1, 0, 0, 0] -> orange2.7.R.B
        row = self.as_int(arr[0], "orange row")
        tree = self.as_int(arr[1], "orange tree")
        side_code = self.as_int(arr[2], "orange side")
        height_code = self.as_int(arr[3], "orange height")

        if row < 1 or row > 4:
            raise ValueError("Orange row must be 1..4, got {}.".format(row))
        if tree < 1 or tree > 8:
            raise ValueError("Orange tree must be 1..8, got {}.".format(tree))
        if side_code not in (0, 1):
            raise ValueError("Orange side must be 0 for L or 1 for R, got {}.".format(side_code))
        if height_code not in (0, 1):
            raise ValueError("Orange height must be 0 for B or 1 for T, got {}.".format(height_code))

        side = "L" if side_code == 0 else "R"
        height = "B" if height_code == 0 else "T"
        return "orange{}.{}.{}.{}".format(row, tree, side, height)

    def assert_orange_available_here(self, orange_name):
        current_node, _ = self.current_node()
        fruit_options = workflow.FRUIT_OPTIONS.get(current_node, [])

        if orange_name not in fruit_options:
            raise ValueError(
                "{} is not available at current node {}. Options: {}".format(
                    orange_name,
                    workflow.display_node_name(self.robot_key, current_node),
                    ", ".join(fruit_options) if fruit_options else "none",
                )
            )

        if orange_name in workflow.UNRIPE_ORANGES:
            raise ValueError("{} is configured as unripe.".format(orange_name))

    def current_node(self):
        pose = workflow.get_current_pose(self.robot_ns, timeout=5.0)
        return workflow.detect_current_node(self.robot_key, pose)

    def run_orange_script_step(
        self,
        orange_name,
        step_label,
        script_name,
        extra_params,
        timeout,
        allow_failure_result=False,
    ):
        if timeout <= 0.0:
            raise RuntimeError("{} timed out before starting.".format(step_label))

        private_params = {"orange": orange_name}
        private_params.update(extra_params)

        moveit_planning_time_sec = 0.0
        moveit_planning_done_time = 0.0
        step_result_time = 0.0
        if script_name == "moveit_target_reach.py":
            workflow.ensure_moveit_planning_time_subscriber(self.robot_ns)
            workflow.ensure_moveit_planning_done_time_subscriber(self.robot_ns)

        rospy.loginfo("%s for %s", step_label, orange_name)
        process_handle = None
        try:
            step_start_time = time.monotonic()
            process_handle = workflow.launch_ros_script(script_name, private_params)
            step_succeeded = workflow.wait_for_orange_action_result(
                self.robot_ns,
                process_handle,
                step_label,
                timeout=timeout,
            )
            step_result_time = time.monotonic()
            if script_name == "moveit_target_reach.py":
                moveit_planning_time_sec = workflow.get_moveit_planning_time_since(
                    self.robot_ns,
                    step_start_time,
                )
                moveit_planning_done_time = workflow.get_moveit_planning_done_time_since(
                    self.robot_ns,
                    step_start_time,
                )
            rospy.sleep(workflow.STEP_SHUTDOWN_DELAY)
        finally:
            workflow.shutdown_process(process_handle)

        if not step_succeeded and not allow_failure_result:
            raise RuntimeError("{} reported orange_action=False.".format(step_label))

        return step_succeeded, moveit_planning_time_sec, moveit_planning_done_time, step_result_time

    def wait_for_navigation_finish(self, process_handle, timeout):
        status_topic = "{}/path_nav/status".format(self.robot_ns)
        deadline = time.time() + timeout
        last_status = None

        while not rospy.is_shutdown():
            remaining = deadline - time.time()
            if remaining <= 0.0:
                return False, "Timed out waiting for navigation after {:.1f}s.".format(timeout)

            if process_handle is not None and process_handle.poll() is not None:
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

    def wait_for_process(self, process_handle, timeout, process_name):
        deadline = time.time() + timeout

        while not rospy.is_shutdown():
            return_code = process_handle.poll()
            if return_code is not None:
                if return_code == 0:
                    return True, "{} completed.".format(process_name)
                return False, "{} exited with code {}.".format(process_name, return_code)

            remaining = deadline - time.time()
            if remaining <= 0.0:
                return False, "Timed out waiting for {} after {:.1f}s.".format(process_name, timeout)

            rospy.sleep(min(0.1, remaining))

        return False, "ROS shutdown while waiting for {}.".format(process_name)

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
    rospy.spin()


if __name__ == "__main__":
    main()
