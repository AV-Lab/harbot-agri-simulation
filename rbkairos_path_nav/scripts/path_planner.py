#!/usr/bin/env python3

import os
import json
import math
import rospy
import rospkg
from collections import deque
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PathPlanner(object):
    def __init__(self):
        package_path = rospkg.RosPack().get_path('rbkairos_path_nav')
        namespace = rospy.get_namespace().strip('/')
        self.robot_namespace = namespace or 'robot'
        self.is_robot_a = self.robot_namespace in ('robot', 'robot_a')
        self.other_robot_namespace = 'robot_b' if self.is_robot_a else 'robot'

        self.path_folder = rospy.get_param('~path_folder', 'paths')
        self.path_topic = rospy.get_param('~path_topic', 'path_nav/path')
        self.path_name = rospy.get_param('~path_name', 'recorded_path.json')
        self.status_topic = rospy.get_param('~status_topic', 'path_nav/status')
        self.odom_topic = rospy.get_param('~odom_topic', 'robotnik_base_control/odom_gt')
        self.other_odom_topic = rospy.get_param(
            '~other_odom_topic',
            '/{}/robotnik_base_control/odom_gt'.format(self.other_robot_namespace)
        )
        self.other_path_topic = rospy.get_param(
            '~other_path_topic',
            '/{}/path_nav/path'.format(self.other_robot_namespace)
        )
        self.proceed_flag_topic = rospy.get_param('~proceed_flag_topic', 'path_nav/proceed_flag')
        self.desired_speed_topic = rospy.get_param('~desired_speed_topic', 'path_nav/desired_speed')
        self.reverse_mode_topic = rospy.get_param('~reverse_mode_topic', 'path_nav/reverse_mode')
        self.path_clearance_radius = float(rospy.get_param('~path_clearance_radius', 0.80))
        self.shared_area_approach_distance = float(
            rospy.get_param('~shared_area_approach_distance', self.path_clearance_radius * 2.0)
        )
        self.clear_confirmation_seconds = float(rospy.get_param('~clear_confirmation_seconds', 2.5))
        self.priority_time_window = float(rospy.get_param('~priority_time_window', 0.5))
        self.priority_distance_margin = float(rospy.get_param('~priority_distance_margin', 0.15))

        self.path = deque()
        self.status = 'IDLE'
        self.frame_id = 'odom'
        self.proceed_flag = False
        self.requested_proceed_flag = False
        self.paused_at_stop = False
        self.blocked_by_safety = False
        self.safety_waiting_clear = False
        self.safety_block_reason = ''
        self.area_clear_since = None
        self.goal_tolerance = 0.5
        self.vehicle_position = []
        self.other_vehicle_position = []
        self.other_path_points = []
        self.current_path_file = ''
        self.reverse_mode = False
        self.own_shared_entry_time = None
        self.other_shared_entry_time = None
        self.previous_self_in_shared = False
        self.previous_other_in_shared = False
        self.ignored_proceed_echo = None

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.speed_pub = rospy.Publisher(self.desired_speed_topic, Float64, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.proceed_pub = rospy.Publisher(self.proceed_flag_topic, Bool, queue_size=1, latch=True)
        self.reverse_mode_pub = rospy.Publisher(self.reverse_mode_topic, Bool, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.other_odom_topic, Odometry, self.other_odom_callback, queue_size=10)
        rospy.Subscriber(self.other_path_topic, Path, self.other_path_callback, queue_size=1)
        rospy.Subscriber(self.proceed_flag_topic, Bool, self.proceed_callback, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.publish_outputs)

        if self.path_name:
            self.load_path_file(os.path.join(package_path, self.path_folder, self.path_name))

        self.update_status()

    def is_reverse_mode(self, speed):
        return speed < 0

    def set_status(self, status_text):
        if self.status == status_text:
            return
        self.status = status_text
        self.status_pub.publish(String(data=self.status))

    def clear_safety_state(self):
        self.blocked_by_safety = False
        self.safety_waiting_clear = False
        self.area_clear_since = None
        self.safety_block_reason = ''
        self.own_shared_entry_time = None
        self.other_shared_entry_time = None
        self.previous_self_in_shared = False
        self.previous_other_in_shared = False

    def update_status(self):
        if not self.path:
            if self.current_path_file:
                self.set_status('FINISHED')
            else:
                self.set_status('IDLE')
        elif self.requested_proceed_flag and self.safety_waiting_clear:
            self.set_status('WAITING_FOR_CLEAR_CONFIRMATION')
        elif self.requested_proceed_flag and self.blocked_by_safety:
            self.set_status('WAITING_FOR_PATH_CLEAR')
        elif self.proceed_flag:
            self.set_status('RUNNING')
        elif self.paused_at_stop:
            self.set_status('STOPPED_AT_POINT')
        else:
            self.set_status('STOPPED')

    def odom_callback(self, msg):
        self.vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw(msg)
        ]
        self.frame_id = msg.header.frame_id or 'odom'

    def other_odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.other_vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ]

    def other_path_callback(self, msg):
        self.other_path_points = []
        for pose in msg.poses:
            q = pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.other_path_points.append([
                pose.pose.position.x,
                pose.pose.position.y,
                yaw
            ])

    def proceed_callback(self, msg):
        if self.ignored_proceed_echo is not None and msg.data == self.ignored_proceed_echo:
            self.ignored_proceed_echo = None
            return

        self.requested_proceed_flag = msg.data
        if self.requested_proceed_flag:
            self.paused_at_stop = False
        else:
            self.clear_safety_state()
        self.apply_effective_proceed_flag()
        self.update_status()

    def publish_proceed_flag(self, proceed_value):
        if self.proceed_flag == proceed_value:
            return
        self.proceed_flag = proceed_value
        self.ignored_proceed_echo = proceed_value
        self.proceed_pub.publish(Bool(data=self.proceed_flag))

    def apply_effective_proceed_flag(self):
        should_proceed = (
            self.requested_proceed_flag
            and not self.paused_at_stop
            and not self.blocked_by_safety
            and not self.safety_waiting_clear
            and bool(self.path)
        )
        self.publish_proceed_flag(should_proceed)

    def load_path_file(self, file_path):
        if not os.path.isfile(file_path):
            rospy.logwarn('Path file does not exist: %s', file_path)
            return

        with open(file_path, 'r', encoding='utf-8') as file_handle:
            raw_path = json.load(file_handle)

        loaded_path = deque()
        for point in raw_path:
            loaded_path.append([
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(point[3]),
                float(point[4]),
                float(point[5])
            ])

        self.path = loaded_path
        self.reverse_mode = self.is_reverse_mode(self.path[0][3]) if self.path else False
        rospy.loginfo('Reverse mode: %s', self.reverse_mode)
        self.reverse_mode_pub.publish(Bool(data=self.reverse_mode))
        self.current_path_file = file_path
        self.requested_proceed_flag = False
        self.paused_at_stop = False
        self.clear_safety_state()
        self.publish_proceed_flag(False)

        if self.path:
            rospy.loginfo('Loaded %d waypoints from %s', len(self.path), os.path.basename(file_path))
        else:
            rospy.logwarn('No valid waypoints found in %s', file_path)

        self.update_status()

    def get_yaw(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw if not self.reverse_mode else yaw + math.pi

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

            if len(self.path) > 1:
                if self.path[0][5] == 1.0 and self.path[1][5] != 1.0:
                    self.requested_proceed_flag = False
                    self.paused_at_stop = True
                    self.publish_proceed_flag(False)
                    self.path.popleft()

        elif len(self.path) == 1:
            current_goal_distance = math.hypot(
                self.path[0][0] - self.vehicle_position[0],
                self.path[0][1] - self.vehicle_position[1]
            )
            if current_goal_distance < self.goal_tolerance:
                self.path.popleft()

    def get_desired_speed(self):
        if not self.proceed_flag or not self.path:
            return 0.0
        if self.path[0][5] == 1.0:
            return 0.0
        return self.path[0][3]

    def path_as_points(self):
        return [[point[0], point[1], point[2]] for point in self.path]

    def pose_inside_points(self, points, pose, radius):
        if not points or not pose:
            return False

        for point in points:
            if math.hypot(point[0] - pose[0], point[1] - pose[1]) <= radius:
                return True
        return False

    def distance_to_points(self, points, pose):
        if not points or not pose:
            return float('inf')
        return min(math.hypot(point[0] - pose[0], point[1] - pose[1]) for point in points)

    def build_shared_area_points(self):
        own_points = self.path_as_points()
        if not own_points or not self.other_path_points:
            return []

        shared_points = []
        seen_points = set()
        match_distance = self.path_clearance_radius * 2.0

        for own_point in own_points:
            for other_point in self.other_path_points:
                if math.hypot(own_point[0] - other_point[0], own_point[1] - other_point[1]) > match_distance:
                    continue
                mid_x = 0.5 * (own_point[0] + other_point[0])
                mid_y = 0.5 * (own_point[1] + other_point[1])
                key = (round(mid_x, 3), round(mid_y, 3))
                if key not in seen_points:
                    seen_points.add(key)
                    shared_points.append([mid_x, mid_y, 0.0])
                break

        return shared_points

    def update_shared_entry_times(self, self_in_shared, other_in_shared):
        now = rospy.get_time()

        if self_in_shared and not self.previous_self_in_shared:
            self.own_shared_entry_time = now
        elif not self_in_shared:
            self.own_shared_entry_time = None

        if other_in_shared and not self.previous_other_in_shared:
            self.other_shared_entry_time = now
        elif not other_in_shared:
            self.other_shared_entry_time = None

        self.previous_self_in_shared = self_in_shared
        self.previous_other_in_shared = other_in_shared

    def other_has_priority(self, shared_points):
        if not shared_points or not self.other_vehicle_position or not self.vehicle_position:
            return False

        self_in_shared = self.pose_inside_points(shared_points, self.vehicle_position, self.path_clearance_radius)
        other_in_shared = self.pose_inside_points(shared_points, self.other_vehicle_position, self.path_clearance_radius)
        self.update_shared_entry_times(self_in_shared, other_in_shared)

        if other_in_shared and not self_in_shared:
            return True
        if self_in_shared and not other_in_shared:
            return False
        if self_in_shared and other_in_shared:
            if self.own_shared_entry_time is not None and self.other_shared_entry_time is not None:
                time_gap = abs(self.own_shared_entry_time - self.other_shared_entry_time)
                if time_gap <= self.priority_time_window:
                    return not self.is_robot_a
                return self.other_shared_entry_time < self.own_shared_entry_time
            return not self.is_robot_a

        self_distance = self.distance_to_points(shared_points, self.vehicle_position)
        other_distance = self.distance_to_points(shared_points, self.other_vehicle_position)
        self_near = self_distance <= self.shared_area_approach_distance
        other_near = other_distance <= self.shared_area_approach_distance

        if not (self_near and other_near):
            return False

        if abs(self_distance - other_distance) <= self.priority_distance_margin:
            return not self.is_robot_a
        return other_distance < self_distance

    def update_safety_gate(self):
        if not self.path or not self.vehicle_position:
            self.clear_safety_state()
            return

        if not self.requested_proceed_flag:
            self.clear_safety_state()
            return

        own_points = self.path_as_points()
        shared_points = self.build_shared_area_points()
        other_in_my_path = self.pose_inside_points(own_points, self.other_vehicle_position, self.path_clearance_radius)
        self_near_shared = (
            bool(shared_points)
            and self.distance_to_points(shared_points, self.vehicle_position) <= self.shared_area_approach_distance
        )
        other_priority = self.other_has_priority(shared_points)

        should_block = False
        block_reason = ''

        if other_in_my_path:
            if other_priority or not shared_points:
                should_block = True
                block_reason = 'other robot is inside this path area'
        elif self_near_shared and other_priority:
            should_block = True
            block_reason = 'other robot has priority for the shared path area'

        if should_block:
            self.blocked_by_safety = True
            self.safety_waiting_clear = False
            self.area_clear_since = None
            self.safety_block_reason = block_reason
            return

        if self.blocked_by_safety:
            now = rospy.get_time()
            if self.area_clear_since is None:
                self.area_clear_since = now
                self.safety_waiting_clear = True
                self.safety_block_reason = 'waiting to confirm the path area stays clear'
                return
            if now - self.area_clear_since < self.clear_confirmation_seconds:
                self.safety_waiting_clear = True
                self.safety_block_reason = 'waiting to confirm the path area stays clear'
                return

        self.blocked_by_safety = False
        self.safety_waiting_clear = False
        self.area_clear_since = None
        self.safety_block_reason = ''

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
        self.update_path()
        self.update_safety_gate()
        self.apply_effective_proceed_flag()
        self.update_status()
        self.path_pub.publish(self.build_path_message())
        self.speed_pub.publish(Float64(data=self.get_desired_speed()))


def main():
    rospy.init_node('path_planner')
    PathPlanner()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
