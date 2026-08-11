#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class PathController(object):
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'robotnik_base_control/cmd_vel')
        self.odom_topic = rospy.get_param('~odom_topic', 'robotnik_base_control/odom')
        self.path_topic = rospy.get_param('~path_topic', 'path_nav/path')
        self.desired_speed_topic = rospy.get_param('~desired_speed_topic', 'path_nav/desired_speed')
        self.reverse_mode_topic = rospy.get_param('~reverse_mode_topic', 'path_nav/reverse_mode')
        self.lookahead_point_index = int(rospy.get_param('~lookahead_point_index', 4))
        self.linear_gain = float(rospy.get_param('~linear_gain', 10.0))
        self.heading_gain = float(rospy.get_param('~heading_gain', 10.0))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 3.5))
        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 0.20))

        self.desired_speed = 0.0
        self.vehicle_position = None
        self.path_points = []
        self.reverse_mode = False

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.desired_speed_topic, Float64, self.speed_callback, queue_size=1)
        rospy.Subscriber(self.reverse_mode_topic, Bool, self.reverse_mode_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_cmd)
 
    def reverse_mode_callback(self, msg):
        self.reverse_mode = msg.data
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ]

    def path_callback(self, msg):
        self.path_points = []
        for pose in msg.poses:
            q = pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.path_points.append([
                pose.pose.position.x,
                pose.pose.position.y,
                yaw
            ])

    def speed_callback(self, msg):
        self.desired_speed = msg.data

    def publish_cmd(self, _event):
        if self.vehicle_position is None or not self.path_points:
            self.cmd_pub.publish(Twist())
            return

        index = max(0, min(len(self.path_points) - 1, self.lookahead_point_index - 1))
        lookahead_point = self.path_points[index]

        # Compute the target position error in the global frame.
        dx = lookahead_point[0] - self.vehicle_position[0]
        dy = lookahead_point[1] - self.vehicle_position[1]
        
        vehicle_yaw = self.vehicle_position[2]
        if self.reverse_mode:
            yaw = vehicle_yaw + math.pi
        else:
            yaw = vehicle_yaw

                
        # Project the world-frame error onto the robot's local axes using dot products.
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy   # local_x = (dx, dy) · (cos(yaw), sin(yaw))
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy  # local_y = (dx, dy) · (-sin(yaw), cos(yaw))
        
        # Proportional velocity command: larger local position error -> larger commanded speed.
        vx = self.linear_gain * local_x if not self.reverse_mode else -self.linear_gain * local_x
        vy = self.linear_gain * local_y if not self.reverse_mode else -self.linear_gain * local_y
        
        # Compute the magnitude of the linear velocity vector.
        speed_norm = math.hypot(vx, vy)
        
        # Scale vx and vy so the total linear speed stays within the limit.
        speed_limit = abs(self.desired_speed)
        if speed_norm > 1e-6 and speed_norm > speed_limit:
            scale = speed_limit / speed_norm
            vx *= scale
            vy *= scale        
            
        # Compute the heading error between the target direction and current robot yaw.
        heading_target =  lookahead_point[2]
            
        heading_error = self.normalize_angle(heading_target - yaw)
        raw_vz = self.heading_gain * heading_error
        vz = max(-self.max_angular_speed, min(raw_vz, self.max_angular_speed))

        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.angular.z = vz
        self.cmd_pub.publish(cmd_msg)


def main():
    rospy.init_node('path_controller')
    PathController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
