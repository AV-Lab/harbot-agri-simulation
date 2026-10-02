#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Receive the global path from the planner.
# - Track progress and publish the remaining path as the local path.
# - Publish the original waypoint index and whether movement is allowed.
# - Detect shared areas between both robots' remaining paths.
# - Allow a robot already inside a shared area to continue.
# - Stop an approaching robot when the shared area is occupied.
# - Give Robot A priority when both robots approach together.
# - Use nearby path blocking as a final emergency safety check.
# ******************************************************************************


import math
import rospy
import threading
from collections import deque
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, UInt32
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PathFollower:
    def __init__(self):
        self.status_topic = rospy.get_param("~status_topic", "path_nav/status")
        self.robot_safety_radius = float(rospy.get_param("~robot_safety_radius", 0.72))
        self.odom_topic = rospy.get_param("~odom_topic", "robotnik_base_control/odom_gt")
        self.local_path_topic = rospy.get_param("~local_path_topic", "path_nav/local_path")
        self.path_index_topic = rospy.get_param("~path_index_topic", "path_nav/path_index")
        self.global_path_topic = rospy.get_param("~global_path_topic", "path_nav/global_path")
        self.proceed_flag_topic = rospy.get_param("~proceed_flag_topic", "path_nav/proceed_flag")
        self.movement_allowed_topic = rospy.get_param("~movement_allowed_topic", "path_nav/movement_allowed")

        self.robot_namespace = rospy.get_namespace().strip("/")
        self.is_robot_a = self.robot_namespace in ("robot", "robot_a")
        self.other_robot_namespace = "robot_b" if self.is_robot_a else "robot"
        self.other_odom_topic = "/{}/{}".format(
            self.other_robot_namespace,
            self.odom_topic.strip("/")
        )
        self.other_global_path_topic = "/{}/{}".format(
            self.other_robot_namespace,
            self.global_path_topic.strip("/")
        )

        self.status = None
        self.path = deque()
        self.path_index = 0
        self.frame_id = "odom"
        self.goal_tolerance = 0.5
        self.vehicle_position = []
        self.run_requested = False
        self.other_path_points = []
        self.has_received_path = False
        self.other_vehicle_position = []
        self.path_lock = threading.Lock()

        self.local_path_pub = rospy.Publisher(self.local_path_topic, Path, queue_size=1)
        self.path_index_pub = rospy.Publisher(self.path_index_topic, UInt32, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.movement_allowed_pub = rospy.Publisher(self.movement_allowed_topic, Bool, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.proceed_flag_topic, Bool, self.proceed_callback, queue_size=1)
        rospy.Subscriber(self.global_path_topic, Path, self.global_path_callback, queue_size=1)
        rospy.Subscriber(self.other_odom_topic, Odometry, self.other_odom_callback, queue_size=10)
        rospy.Subscriber(self.other_global_path_topic, Path, self.other_global_path_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_outputs)

        self.update_status()

    def set_status(self, status_text):
        if self.status == status_text:
            return
        self.status = status_text
        self.status_pub.publish(String(data=self.status))

    def update_status(self, safety_stop=False):
        if not self.path:
            if self.has_received_path:
                self.set_status("FINISHED")
            else:
                self.set_status("IDLE")
        elif self.run_requested and safety_stop:
            self.set_status("WAITING_FOR_PATH_CLEAR")
        elif self.run_requested:
            self.set_status("RUNNING")
        else:
            self.set_status("STOPPED")

    def odom_callback(self, msg):
        self.vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ]
        self.frame_id = msg.header.frame_id or "odom"

    def other_odom_callback(self, msg):
        self.other_vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ]

    def other_global_path_callback(self, msg):
        new_path = [
            [
                pose.pose.position.x,
                pose.pose.position.y,
            ]

            for pose in msg.poses
        ]

        if new_path:
            with self.path_lock:
                self.other_path_points = new_path

    def proceed_callback(self, msg):
        with self.path_lock:
            self.run_requested = msg.data

    def global_path_callback(self, msg):
        loaded_path = deque()
        for pose_stamped in msg.poses:
            q = pose_stamped.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            loaded_path.append([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                yaw,
            ])

        with self.path_lock:
            self.path = loaded_path
            self.path_index = 0
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id
            self.has_received_path = True

            if self.path:
                rospy.loginfo("Received global path with %d waypoints", len(self.path))
            else:
                rospy.logwarn("Received an empty global path")

            self.update_status()

    def update_path(self):
        if not self.vehicle_position or not self.path:
            return

        if len(self.path) > 1:
            current_goal_distance = math.hypot(
                self.path[0][0] - self.vehicle_position[0],
                self.path[0][1] - self.vehicle_position[1]
            )
            next_goal_distance = math.hypot(
                self.path[1][0] - self.vehicle_position[0],
                self.path[1][1] - self.vehicle_position[1]
            )

            if next_goal_distance < current_goal_distance:
                self.path.popleft()
                self.path_index += 1

        elif len(self.path) == 1:
            current_goal_distance = math.hypot(
                self.path[0][0] - self.vehicle_position[0],
                self.path[0][1] - self.vehicle_position[1]
            )
            if current_goal_distance < self.goal_tolerance:
                self.path.popleft()
                self.path_index += 1

    def distance_to_points(self, points, pose):
        if not points or not pose:
            return float("inf")
        return min(math.hypot(point[0] - pose[0], point[1] - pose[1]) for point in points)

    def build_shared_area_points(self):
        own_points = [[point[0], point[1]] for point in self.path]
        if not own_points or not self.other_path_points:
            return []

        shared_points = []
        seen_points = set()
        collision_distance = self.robot_safety_radius * 2.0

        for own_point in own_points:
            for other_point in self.other_path_points:
                if math.hypot(
                    own_point[0] - other_point[0],
                    own_point[1] - other_point[1],
                ) > collision_distance:
                    continue
                mid_x = 0.5 * (own_point[0] + other_point[0])
                mid_y = 0.5 * (own_point[1] + other_point[1])
                key = (round(mid_x, 3), round(mid_y, 3))
                if key not in seen_points:
                    seen_points.add(key)
                    shared_points.append([mid_x, mid_y])
                break

        return shared_points

    def should_stop_for_safety(self):
        if (
            not self.run_requested
            or not self.path
            or not self.vehicle_position
            or not self.other_vehicle_position
        ):
            return False

        own_points = [[point[0], point[1]] for point in self.path]
        shared_points = self.build_shared_area_points()
        collision_distance = self.robot_safety_radius * 2.0

        self_distance = self.distance_to_points(shared_points, self.vehicle_position)
        other_distance = self.distance_to_points(shared_points, self.other_vehicle_position)
        self_inside = self_distance <= self.robot_safety_radius
        other_inside = other_distance <= self.robot_safety_radius

        if other_inside and not self_inside:
            return self_distance <= collision_distance
        if self_inside and not other_inside:
            return False

        both_approaching = (
            self_distance <= collision_distance
            and other_distance <= collision_distance
        )
        if both_approaching:
            return not self.is_robot_a

        other_robot_distance = math.hypot(
            self.vehicle_position[0] - self.other_vehicle_position[0],
            self.vehicle_position[1] - self.other_vehicle_position[1],
        )
        other_blocks_nearby_path = (
            other_robot_distance <= collision_distance
            and self.distance_to_points(own_points, self.other_vehicle_position)
            <= collision_distance
        )

        return other_blocks_nearby_path

    def build_path_message(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id

        for point in self.path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            _, _, qz, qw = quaternion_from_euler(0.0, 0.0, point[2])
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            path_msg.poses.append(pose_stamped)

        return path_msg

    def publish_outputs(self, _event):
        with self.path_lock:
            self.update_path()
            safety_stop = self.should_stop_for_safety()
            movement_allowed = bool(
                self.run_requested and self.path and not safety_stop
            )
            self.update_status(safety_stop)
            self.local_path_pub.publish(self.build_path_message())
            self.path_index_pub.publish(UInt32(data=self.path_index))
            self.movement_allowed_pub.publish(Bool(data=movement_allowed))


def main():
    rospy.init_node("path_follower")
    PathFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
