#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Receive the local path, desired speed, and robot odometry.
# - Select a lookahead point from the remaining path.
# - Transform the target position error into the robot's local frame.
# - Calculate proportional linear and angular velocity commands.
# - Limit the linear and angular velocities to their configured values.
# - Support both forward and reverse path following.
# - Publish zero velocity when stopped or when no valid path is available.
# ******************************************************************************


import math
import rospy
import threading
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class HolonomicController:
    def __init__(self):
        self.linear_gain = float(rospy.get_param("~linear_gain"))
        self.heading_gain = float(rospy.get_param("~heading_gain"))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed"))
        self.lookahead_point_index = int(rospy.get_param("~lookahead_point_index"))
        self.odom_topic = rospy.get_param("~odom_topic", "robotnik_base_control/odom")
        self.local_path_topic = rospy.get_param("~local_path_topic", "path_nav/local_path")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "robotnik_base_control/cmd_vel")
        self.desired_speed_topic = rospy.get_param("~desired_speed_topic", "path_nav/desired_speed")

        self.path_points = []
        self.desired_speed = 0.0
        self.reverse_mode = False
        self.vehicle_position = None
        self.state_lock = threading.Lock()

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.local_path_topic, Path, self.local_path_callback, queue_size=1)
        rospy.Subscriber(self.desired_speed_topic, Float64, self.desired_speed_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.state_lock:
            self.vehicle_position = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw
            ]

    def local_path_callback(self, msg):
        path_points = []
        for pose in msg.poses:
            q = pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            path_points.append([
                pose.pose.position.x,
                pose.pose.position.y,
                yaw
            ])

        with self.state_lock:
            if not path_points:
                self.desired_speed = 0.0
            self.path_points = path_points

    def desired_speed_callback(self, msg):
        with self.state_lock:
            self.desired_speed = (
                msg.data
                if self.path_points and math.isfinite(msg.data)
                else 0.0
            )
            if self.desired_speed != 0.0:
                self.reverse_mode = self.desired_speed < 0.0

    def publish_cmd(self, _event):
        with self.state_lock:
            self.update_cmd()

    def update_cmd(self):
        if (
            self.vehicle_position is None
            or not self.path_points
            or self.desired_speed == 0.0
        ):
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
        heading_target = lookahead_point[2]

        heading_error = self.normalize_angle(heading_target - yaw)
        raw_vz = self.heading_gain * heading_error
        vz = max(-self.max_angular_speed, min(raw_vz, self.max_angular_speed))

        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.angular.z = vz
        self.cmd_pub.publish(cmd_msg)


def main():
    rospy.init_node("holonomic_controller")
    HolonomicController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
