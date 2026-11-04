#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Move the arm to the scan pose for the requested orange.
# - Wait for the detected orange position and transform it into the planning frame.
# - Shift the detected point by the orange radius, away from the robot, to its center.
# - Add the target and paired oranges and the stem meshes to the planning scene.
# - Publish RViz markers for the target and the collision objects.
# - Plan and execute the reach with MoveIt.
# - Publish the total planning time, the planning-done time, and orange_action.
# ******************************************************************************


import sys
import time
import rospy
import tf2_ros
import moveit_commander
# Registers PointStamped with tf2, so tf_buffer.transform() accepts it.
import tf2_geometry_msgs  # noqa: F401
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from harbot_harvest import arm, helpers
from trajectory_msgs.msg import JointTrajectory
from moveit_commander import PlanningSceneInterface
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped


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

IDENTITY_ORIENTATION = (0.0, 0.0, 0.0, 1.0)
MESH_ORIENTATION = (0.70710678118, 0.0, 0.0, 0.70710678118)
COLLISION_COLOR = (0.0, 1.0, 0.0, 0.85)


def make_pose_stamped(frame_id, position, orientation=IDENTITY_ORIENTATION):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = orientation
    return pose


def make_marker(frame_id, namespace, marker_id, marker_type, position, scale, color, orientation=IDENTITY_ORIENTATION):
    marker = Marker()
    marker.id = marker_id
    marker.ns = namespace
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.header.frame_id = frame_id
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = orientation
    return marker


class MoveItTargetReach:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.orange_model = rospy.get_param("~orange", "").strip()
        self.planning_frame = helpers.get_setting("planning_frame")
        self.detected_pose_topic = self.robot_ns + "/object_position"
        self.odom_topic = helpers.get_robot_topic(self.robot_ns, "odom_topic")
        self.result_topic = helpers.get_robot_topic(self.robot_ns, "result_topic")
        self.arm_cmd_topic = helpers.get_robot_topic(self.robot_ns, "arm_command_topic")
        self.gripper_cmd_topic = helpers.get_robot_topic(self.robot_ns, "gripper_command_topic")
        self.orange_stem_mesh_path = rospy.get_param("/harbot/planning_scene/orange_stem_mesh_path", "")
        self.greenhouse_stems_mesh_path = rospy.get_param("/harbot/planning_scene/greenhouse_stems_mesh_path", "")

        self.side = None
        self.height = None
        self.diameter = 0.07
        self.latest_odom = None
        self.planning_time = 40.0
        self.planning_attempts = 14
        self.target_position = None
        self.planner_id = "RRTConnect"
        self.position_tolerance = 0.01
        self.detection_timeout = 10.0
        self.no_solution_retries = 14
        self.move_group_wait_timeout = 50.0
        self.moveit_planning_time_sec = 0.0
        self.move_group_connect_retries = 9
        self.arm = arm.FrankaArm(self.robot_ns)
        self.move_group_connect_retry_delay = 2.0
        self.collision_meshes = [
            ("greenhouse_stems_link", self.greenhouse_stems_mesh_path),
            ("orange_stem", self.orange_stem_mesh_path),
        ]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

    def load_orange_detection(self):
        rospy.loginfo("Waiting for detected orange pose on %s", self.detected_pose_topic)

        try:
            msg = rospy.wait_for_message(self.detected_pose_topic, PointStamped, timeout=self.detection_timeout)
        except rospy.ROSException as exc:
            raise RuntimeError("No detected orange pose received on {} within {:.1f} seconds".format(self.detected_pose_topic, self.detection_timeout)) from exc

        msg = self.tf_buffer.transform(msg, self.planning_frame, rospy.Duration(1.0))
        if self.latest_odom is None:
            raise RuntimeError("No odom received yet")

        # The camera sees the near surface; shift by the radius to get the center.
        offset_x = msg.point.x - self.latest_odom.pose.pose.position.x
        direction = (offset_x > 0) - (offset_x < 0)
        self.target_position = [msg.point.x + (self.diameter / 2.0) * direction, msg.point.y, msg.point.z]

        rospy.loginfo("Detected MoveIt target: [%.5f, %.5f, %.5f]", *self.target_position)

    def paired_orange_position(self):
        # The other orange on the same tree side: below a top orange, above a bottom one.
        sign = -1.0 if self.height == "T" else 1.0
        return [value + sign * delta for value, delta in zip(self.target_position, TOP_FROM_BOTTOM[self.side])]

    def publish_markers(self):
        paired_position = self.paired_orange_position()
        diameter_scale = (self.diameter, self.diameter, self.diameter)

        object_marker = make_marker(self.planning_frame, "detected_object", 0, Marker.SPHERE, self.target_position, diameter_scale, (1.0, 0.45, 0.0, 0.9))
        target_marker = make_marker(self.planning_frame, "target_point", 1, Marker.SPHERE, self.target_position, (0.01, 0.01, 0.01), (1.0, 0.0, 0.0, 1.0))

        collision_markers = MarkerArray()
        for marker_id, position in enumerate((self.target_position, paired_position)):
            marker = make_marker(self.planning_frame, "selected_collision_markers", marker_id, Marker.SPHERE, position, diameter_scale, COLLISION_COLOR)
            collision_markers.markers.append(marker)

        for marker_id, (_, mesh_path) in enumerate(self.collision_meshes, start=2):
            marker = make_marker(self.planning_frame, "selected_collision_markers", marker_id, Marker.MESH_RESOURCE, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), COLLISION_COLOR, MESH_ORIENTATION)
            marker.mesh_resource = "file://" + mesh_path
            collision_markers.markers.append(marker)

        stamp = rospy.Time.now()
        for marker in [object_marker, target_marker] + collision_markers.markers:
            marker.header.stamp = stamp

        self.object_marker_pub.publish(object_marker)
        self.target_marker_pub.publish(target_marker)
        self.collision_marker_pub.publish(collision_markers)

    def add_collision_objects(self, scene):
        paired_position = self.paired_orange_position()
        radius = self.diameter / 2.0

        scene.add_sphere("detected_object", make_pose_stamped(self.planning_frame, self.target_position), radius=radius)
        scene.add_sphere("paired_detected_object", make_pose_stamped(self.planning_frame, paired_position), radius=radius)

        for object_id, mesh_path in self.collision_meshes:
            scene.add_mesh(object_id, make_pose_stamped(self.planning_frame, (0.0, 0.0, 0.0), MESH_ORIENTATION), mesh_path, size=(1.0, 1.0, 1.0))

        rospy.loginfo("Collision objects for orange added to MoveIt planning scene")

    def plan_to_target(self, group):
        for attempt in range(1, self.no_solution_retries + 2):
            rospy.loginfo("Planning to target (attempt %d/%d)", attempt, self.no_solution_retries + 1)

            planning_start = time.monotonic()
            success, plan, _, _ = group.plan()  # (success, plan, planning_time, error_code)
            self.moveit_planning_time_sec += time.monotonic() - planning_start
            if success:
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
            rospy.logerr("Parameter ~orange is required")
            return

        row, tree, self.side, self.height = helpers.parse_orange(self.orange_model)

        rospy.sleep(0.5)
        self.arm_pub.publish(arm.joint_command(self.arm.arm_joints, self.scan_pose(row, tree), 3.0))
        rospy.loginfo("Moved arm to scan pose for %s", self.orange_model)
        rospy.sleep(5.0)

        try:
            self.load_orange_detection()
        except Exception as exc:
            rospy.logerr("Failed to get detected orange pose: %s", exc)
            return

        self.publish_markers()
        success = self.move_arm_to_target()
        self.planning_time_pub.publish(Float64(data=self.moveit_planning_time_sec))
        self.result_pub.publish(Bool(data=success))


def main():
    rospy.init_node("moveit_target_reach", anonymous=True)
    MoveItTargetReach().run()


if __name__ == "__main__":
    main()
