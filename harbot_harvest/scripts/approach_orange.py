#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Look up the base x position used to harvest the requested orange.
# - Drive the base along x until it is within the goal tolerance.
# - Rotate the world-frame x velocity into the robot frame using odometry yaw.
# - Stop the base and publish orange_action=True when the goal is reached.
# ******************************************************************************


import math
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from harbot_harvest import helpers
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


# Base x position for each (orange row, tree side) that can be harvested.
APPROACH_X = {
    (1, "R"): -4.5,
    (2, "L"): -3.5,
    (2, "R"): -0.5,
    (3, "L"): 0.5,
    (3, "R"): 3.5,
    (4, "L"): 4.5,
}


class OrangeApproach:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.orange_model = rospy.get_param("~orange", "").strip()
        self.odom_topic = helpers.get_robot_topic(self.robot_ns, "odom_topic")
        self.cmd_topic = helpers.get_robot_topic(self.robot_ns, "cmd_vel_topic")
        self.result_topic = helpers.get_robot_topic(self.robot_ns, "result_topic")

        self.yaw = 0.0
        self.speed = 0.4
        self.done = False
        self.goal_x = None
        self.latest_odom = None
        self.goal_tolerance = 0.005

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.result_pub = rospy.Publisher(self.result_topic, Bool, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.on_shutdown(self.stop_robot)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_cmd)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_odom = msg

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def find_goal_x(self, current_x):
        row, _, side, _ = helpers.parse_orange(self.orange_model)
        # Oranges without an approach position are harvested from where the robot is.
        return APPROACH_X.get((row, side), current_x)

    def publish_cmd(self, _event):
        if self.latest_odom is None or self.done:
            return

        current_x = self.latest_odom.pose.pose.position.x
        if self.goal_x is None:
            self.goal_x = self.find_goal_x(current_x)
            rospy.loginfo("start_x=%.4f  goal_x=%.4f  dx=%.4f", current_x, self.goal_x, self.goal_x - current_x)
            return

        remaining_x = self.goal_x - current_x
        if abs(remaining_x) <= self.goal_tolerance:
            self.done = True
            self.stop_robot()
            rospy.loginfo("Reached near orange")
            self.result_pub.publish(Bool(data=True))
            rospy.sleep(0.1)
            rospy.signal_shutdown("orange approach complete")
            return

        world_vx = self.speed if remaining_x > 0.0 else -self.speed

        cmd = Twist()
        cmd.linear.x = math.cos(self.yaw) * world_vx
        cmd.linear.y = -math.sin(self.yaw) * world_vx
        self.cmd_pub.publish(cmd)


def main():
    rospy.init_node("approach_orange", anonymous=True)
    OrangeApproach()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
