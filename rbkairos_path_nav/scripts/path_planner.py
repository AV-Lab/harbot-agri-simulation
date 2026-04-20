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
        
        self.path_folder = rospy.get_param('~path_folder', 'paths')
        self.path_topic = rospy.get_param('~path_topic', 'path_nav/path')
        self.path_name = rospy.get_param('~path_name', 'recorded_path.json')        
        self.status_topic = rospy.get_param('~status_topic', 'path_nav/status')
        self.odom_topic = rospy.get_param('~odom_topic', 'robotnik_base_control/odom_gt')
        self.proceed_flag_topic = rospy.get_param('~proceed_flag_topic', 'path_nav/proceed_flag')
        self.desired_speed_topic = rospy.get_param('~desired_speed_topic', 'path_nav/desired_speed')
        self.reverse_mode_topic = rospy.get_param('~reverse_mode_topic', 'path_nav/reverse_mode')
        
        self.path = deque()
        self.status = 'IDLE'
        self.frame_id = 'odom'        
        self.proceed_flag = False
        self.paused_at_stop = False
        self.goal_tolerance = 0.5
        self.vehicle_position = []
        self.current_path_file = ''
        self.reverse_mode = False

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.speed_pub = rospy.Publisher(self.desired_speed_topic, Float64, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.proceed_pub = rospy.Publisher(self.proceed_flag_topic, Bool, queue_size=1, latch=True)
        self.reverse_mode_pub = rospy.Publisher(self.reverse_mode_topic, Bool, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
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

    def update_status(self):
        if not self.path:
            if self.current_path_file:
                self.set_status('FINISHED')
            else:
                self.set_status('IDLE')
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
        
    def proceed_callback(self, msg):
        self.proceed_flag = msg.data
        if self.proceed_flag:
            self.paused_at_stop = False
        self.update_status()
        
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
        self.reverse_mode = self.is_reverse_mode(self.path[0][3])
        rospy.loginfo('Reverse mode: %s', self.reverse_mode)
        self.reverse_mode_pub.publish(Bool(data=self.reverse_mode))
        self.current_path_file = file_path
        self.proceed_flag = False

        if self.path:
            rospy.loginfo('Loaded %d waypoints from %s', len(self.path),  os.path.basename(file_path))
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
                if self.path[0][5] == 1.0 and self.path[1][5] != 1.0 :
                    self.proceed_flag = False
                    self.paused_at_stop = True
                    self.proceed_pub.publish(Bool(data=self.proceed_flag))
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