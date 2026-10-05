#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Move the arm to the scan pose for the requested orange (Franka only).
# - Start orange_pose_to_world and wait for the detected orange world position.
# - Offset the target by the orange radius towards the robot.
# - Add the orange supports, oranges, and stem meshes to the planning scene.
# - Publish RViz markers for the target and the collision objects.
# - Attach the orange to the environment, then plan and execute with MoveIt.
# - Publish the total planning time, the planning-done time, and orange_action.
# ******************************************************************************


import os
import sys
import time
import rospy
import moveit_commander
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory
from harbot_harvest import arm, helpers, ros_utils
from moveit_commander import PlanningSceneInterface
from visualization_msgs.msg import Marker, MarkerArray
from harbot_harvest.attachment import ATTACHMENT_TARGET_ENVIRONMENT, ObjectAttachmentClient


# Franka joint positions that point the wrist camera at each group of oranges.
SCAN_POSES = {
    "back_right": {
        "top": [-0.249, -0.914, -0.461, -2.035, 1.494, 0.906, -0.273],
        "bottom": [-0.429, 0.233, 0.030, -1.904, 1.325, 1.211, -1.360],
    },
    "back_left": {
        "top": [-0.125, 0.653, 0.982, -0.622, -2.408, 1.137, 1.831],
        "bottom": [0.163, -1.037, 0.807, -2.724, -0.755, 1.074, 2.606],
    },
    "front_right": {
        "top": [-0.241, -1.294, -0.601, -1.975, 1.576, 0.852, 0.225],
        "bottom": [-0.510, -0.155, -0.050, -2.357, 1.190, 1.105, -1.470],
    },
    "front_left": {
        "top": [-0.734, -0.632, 0.671, -1.777, -1.178, 1.575, 2.058],
        "bottom": [-2.086, -1.062, -1.309, -2.582, -2.480, 4.206, 2.007],
    },
}

# Offset from the bottom orange to the top orange of a tree, for each tree side.
TOP_FROM_BOTTOM = {
    "R": (0.06760, -0.53770, 0.40465),
    "L": (-0.06761, 0.53580, 0.40465),
}

MESH_ORIENTATION = (0.70710678118, 0.0, 0.0, 0.70710678118)
COLLISION_COLOR = (0.0, 1.0, 0.0, 0.85)


class MoveItTargetReach:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.arm_type = rospy.get_param("~arm", "Franka")
        self.orange_model = rospy.get_param("~orange", "").strip()
        self.planning_frame = helpers.get_setting("planning_frame")
        self.odom_topic = helpers.get_robot_topic(self.robot_ns, "odom_topic")
        self.launch_detection = bool(rospy.get_param("~launch_detection", True))
        self.result_topic = helpers.get_robot_topic(self.robot_ns, "result_topic")
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 10.0))
        self.arm_cmd_topic = helpers.get_robot_topic(self.robot_ns, "arm_command_topic")
        self.no_solution_retries = max(0, int(rospy.get_param("~no_solution_retries", 14)))
        self.detection_startup_delay = float(rospy.get_param("~detection_startup_delay", 0.1))
        self.move_group_wait_timeout = float(rospy.get_param("~move_group_wait_timeout", 50.0))
        self.gripper_cmd_topic = helpers.get_robot_topic(self.robot_ns, "gripper_command_topic")
        self.orange_stem_mesh_path = rospy.get_param("/harbot/planning_scene/orange_stem_mesh_path", "")
        self.move_group_connect_retries = max(0, int(rospy.get_param("~move_group_connect_retries", 9)))
        self.move_group_connect_retry_delay = float(rospy.get_param("~move_group_connect_retry_delay", 2.0))
        self.greenhouse_stems_mesh_path = rospy.get_param("/harbot/planning_scene/greenhouse_stems_mesh_path", "")
        self.detected_pose_topic = rospy.get_param("~detected_pose_topic", self.robot_ns + "/detected_orange_pose_world")

        self.side = None
        self.height = None
        self.diameter = 0.07
        self.latest_odom = None
        self.planning_time = 40.0
        self.planning_attempts = 14
        self.target_position = None
        self.planner_id = "RRTConnect"
        self.position_tolerance = 0.01
        self.moveit_planning_time_sec = 0.0
        self.arm = arm.get_arm_config(self.arm_type, self.robot_ns)
        self.attachment_client = ObjectAttachmentClient(self.robot_ns)
        self.collision_meshes = [
            ("greenhouse_stems_link", self.greenhouse_stems_mesh_path, "~add_greenhouse_stems_collision"),
            ("orange_stem", self.orange_stem_mesh_path, "~add_orange_stem_collision"),
        ]

        self.result_pub = rospy.Publisher(self.result_topic, Bool, queue_size=1)
        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)
        self.target_marker_pub = rospy.Publisher(self.robot_ns + "/target_point_marker", Marker, queue_size=1, latch=True)
        self.object_marker_pub = rospy.Publisher(self.robot_ns + "/detected_object_marker", Marker, queue_size=1, latch=True)
        self.planning_time_pub = rospy.Publisher(self.robot_ns + "/moveit_planning_time_sec", Float64, queue_size=1, latch=True)
        self.planning_done_time_pub = rospy.Publisher(self.robot_ns + "/moveit_planning_done_time", Float64, queue_size=1, latch=True)
        self.collision_marker_pub = rospy.Publisher(self.robot_ns + "/selected_collision_markers", MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def scan_pose(self, row, tree):
        if self.side == "L":
            pose_group = "back_right" if row in (1, 4) or (row == 3 and tree <= 4) else "front_left"
        else:
            pose_group = "back_left" if row == 3 or (row == 2 and tree <= 4) else "front_right"
        return SCAN_POSES[pose_group]["top" if self.height == "T" else "bottom"]

    def start_detection(self):
        if not self.launch_detection:
            return None

        params = {"robot_ns": self.robot_ns, "target_frame": self.planning_frame}
        process_handle = ros_utils.start_process(ros_utils.rosrun_command("harbot_harvest", "orange_pose_to_world.py", params))
        rospy.sleep(self.detection_startup_delay)
        return process_handle

    def load_orange_detection(self):
        rospy.loginfo("Waiting for detected orange pose on %s", self.detected_pose_topic)

        try:
            msg = rospy.wait_for_message(self.detected_pose_topic, PointStamped, timeout=self.detection_timeout)
        except rospy.ROSException as exc:
            raise RuntimeError("No detected orange pose received on {} within {:.1f} seconds".format(self.detected_pose_topic, self.detection_timeout)) from exc

        if msg.header.frame_id != self.planning_frame:
            raise RuntimeError("Expected detected orange pose in frame {}, got {}".format(self.planning_frame, msg.header.frame_id))
        if self.latest_odom is None:
            raise RuntimeError("No odom received yet")

        # Aim at the orange surface facing the robot instead of its center.
        offset_x = msg.point.x - self.latest_odom.pose.pose.position.x
        direction = (offset_x > 0) - (offset_x < 0)
        self.target_position = [msg.point.x + (self.diameter / 2.0) * direction, msg.point.y, msg.point.z]

        rospy.loginfo("Detected MoveIt target: [%.5f, %.5f, %.5f]", *self.target_position)

    def support_positions(self):
        # Returns the top orange, the bottom orange, and the orange paired with the target.
        offset = TOP_FROM_BOTTOM[self.side]
        if self.height == "T":
            top = self.target_position
            bottom = [value - delta for value, delta in zip(top, offset)]
            return top, bottom, bottom

        bottom = self.target_position
        top = [value + delta for value, delta in zip(bottom, offset)]
        return top, bottom, top

    def mesh_available(self, object_id, mesh_path, enabled_param):
        if not bool(rospy.get_param(enabled_param, True)):
            return False
        if not mesh_path:
            rospy.logwarn_once("No mesh path is configured for %s; skipping it", object_id)
            return False
        if not os.path.isfile(mesh_path):
            rospy.logwarn_once("Collision mesh not found for %s: %s", object_id, mesh_path)
            return False
        return True

    def publish_markers(self):
        _, _, paired_position = self.support_positions()
        diameter_scale = (self.diameter, self.diameter, self.diameter)

        object_marker = ros_utils.make_marker(self.planning_frame, "detected_object", 0, Marker.SPHERE, self.target_position, diameter_scale, (1.0, 0.45, 0.0, 0.9))
        target_marker = ros_utils.make_marker(self.planning_frame, "target_point", 1, Marker.SPHERE, self.target_position, (0.01, 0.01, 0.01), (1.0, 0.0, 0.0, 1.0))

        collision_markers = MarkerArray()
        for marker_id, position in enumerate((self.target_position, paired_position)):
            marker = ros_utils.make_marker(self.planning_frame, "selected_collision_markers", marker_id, Marker.SPHERE, position, diameter_scale, COLLISION_COLOR)
            collision_markers.markers.append(marker)

        for marker_id, (object_id, mesh_path, enabled_param) in enumerate(self.collision_meshes, start=2):
            if self.mesh_available(object_id, mesh_path, enabled_param):
                marker = ros_utils.make_marker(self.planning_frame, "selected_collision_markers", marker_id, Marker.MESH_RESOURCE, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), COLLISION_COLOR, MESH_ORIENTATION)
                marker.mesh_resource = "file://" + mesh_path
                collision_markers.markers.append(marker)

        stamp = rospy.Time.now()
        for marker in [object_marker, target_marker] + collision_markers.markers:
            marker.header.stamp = stamp

        self.object_marker_pub.publish(object_marker)
        self.target_marker_pub.publish(target_marker)
        self.collision_marker_pub.publish(collision_markers)

    def add_collision_objects(self, scene):
        top, bottom, paired_position = self.support_positions()
        radius = self.diameter / 2.0

        scene.add_box("top_support_box", ros_utils.make_pose_stamped(self.planning_frame, (top[0], top[1], top[2] - 0.685762)), size=(0.042, 0.042, 1.32421))
        scene.add_box("bottom_support_box", ros_utils.make_pose_stamped(self.planning_frame, (bottom[0], bottom[1], bottom[2] - 0.483475)), size=(0.042, 0.042, 0.919556))
        scene.add_sphere("detected_object", ros_utils.make_pose_stamped(self.planning_frame, self.target_position), radius=radius)
        scene.add_sphere("paired_detected_object", ros_utils.make_pose_stamped(self.planning_frame, paired_position), radius=radius)

        for object_id, mesh_path, enabled_param in self.collision_meshes:
            if self.mesh_available(object_id, mesh_path, enabled_param):
                scene.add_mesh(object_id, ros_utils.make_pose_stamped(self.planning_frame, (0.0, 0.0, 0.0), MESH_ORIENTATION), mesh_path, size=(1.0, 1.0, 1.0))

        rospy.loginfo("Collision objects for orange added to MoveIt planning scene")

    def attach_orange_to_environment(self):
        for _ in range(5):
            if self.attachment_client.set_target(self.orange_model, ATTACHMENT_TARGET_ENVIRONMENT):
                rospy.sleep(0.1)
                return True
            rospy.sleep(0.1)
        return False

    def valid_plan(self, plan_result):
        # MoveIt returns (success, plan, ...) in Noetic and a plain plan in older versions.
        if isinstance(plan_result, tuple):
            if len(plan_result) < 2 or not plan_result[0]:
                return None
            plan_result = plan_result[1]

        trajectory = getattr(plan_result, "joint_trajectory", None)
        return plan_result if trajectory and trajectory.points else None

    def plan_to_target(self, group):
        for attempt in range(1, self.no_solution_retries + 2):
            rospy.loginfo("Planning to target (attempt %d/%d)", attempt, self.no_solution_retries + 1)

            planning_start = time.monotonic()
            plan = self.valid_plan(group.plan())
            self.moveit_planning_time_sec += time.monotonic() - planning_start
            if plan is not None:
                self.planning_done_time_pub.publish(Float64(data=time.monotonic()))
                return plan

            # Open the gripper before replanning in case it is blocking the goal.
            if attempt <= self.no_solution_retries:
                self.gripper_pub.publish(arm.joint_command([self.arm.gripper_joint], [self.arm.gripper_open], 2.0))
                rospy.sleep(0.1)

        return None

    def move_arm_to_target(self):
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            scene = PlanningSceneInterface(ns=self.robot_ns)
            group = arm.create_move_group(self.arm.move_group, self.robot_ns, self.move_group_wait_timeout, self.move_group_connect_retries, self.move_group_connect_retry_delay)
            if not self.attach_orange_to_environment():
                return False

            group.set_planner_id(self.planner_id)
            group.set_planning_time(self.planning_time)
            group.set_num_planning_attempts(self.planning_attempts)
            group.set_goal_position_tolerance(self.position_tolerance)
            rospy.loginfo("Using end-effector: %s | Target: [%.5f, %.5f, %.5f]", self.arm.ee_link, *self.target_position)

            self.add_collision_objects(scene)
            rospy.sleep(1.0)

            group.set_pose_reference_frame(self.planning_frame)
            group.set_position_target(self.target_position, self.arm.ee_link)

            plan = self.plan_to_target(group)
            if plan is None:
                rospy.logerr("MoveIt found no valid plan after retries; not executing")
                success = False
            else:
                success = group.execute(plan, wait=True)
                group.stop()
                if success:
                    rospy.loginfo("Arm reached the target position")

            group.clear_pose_targets()
            return success
        finally:
            moveit_commander.roscpp_shutdown()

    def run(self):
        if not self.orange_model:
            rospy.logerr("Set ~orange to an orange model name, for example orange2.4.R.T")
            return

        try:
            row, tree, self.side, self.height = helpers.parse_orange(self.orange_model)
        except ValueError as exc:
            rospy.logerr("%s", exc)
            return

        rospy.sleep(0.5)
        if str(self.arm_type).lower() == "franka":
            self.arm_pub.publish(arm.joint_command(self.arm.arm_joints, self.scan_pose(row, tree), 3.0))
            rospy.loginfo("Moved arm to scan pose for %s", self.orange_model)
        else:
            rospy.logwarn("Scan pose is only configured for Franka; skipping")
        rospy.sleep(5.0)

        detection_process = None
        try:
            detection_process = self.start_detection()
            self.load_orange_detection()
        except Exception as exc:
            rospy.logerr("Failed to get detected orange pose: %s", exc)
            return
        finally:
            ros_utils.shutdown_process(detection_process, timeout=3.0)

        self.publish_markers()
        success = self.move_arm_to_target()
        self.planning_time_pub.publish(Float64(data=self.moveit_planning_time_sec))
        self.result_pub.publish(Bool(data=success))


def main():
    rospy.init_node("moveit_target_reach", anonymous=True)
    MoveItTargetReach().run()


if __name__ == "__main__":
    main()
