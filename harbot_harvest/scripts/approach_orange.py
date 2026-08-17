#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from harbot_harvest import helpers


class OrangeApproach(object):
    def __init__(self):

        self.robot_ns = helpers.normalize_robot_namespace(
            rospy.get_param('~robot_ns', '/robot')
        )
        self.orange_model = rospy.get_param('~orange', '').strip()
        
        self.dx = 0.0
        self.yaw = 0.0
        self.speed = 0.4
        self.yaw_tolerance = 0.15
        self.goal_tolerance = 0.005
        
        self.done = False
        self.goal_x = None
        self.latest_odom = None


        self.cmd_topic = helpers.get_robot_topic(self.robot_ns, 'cmd_vel_topic')
        self.odom_topic = helpers.get_robot_topic(self.robot_ns, 'odom_topic')
        self.orange_action_topic = helpers.get_robot_topic(self.robot_ns, 'result_topic')
        
        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        
        rospy.on_shutdown(self.stop_robot)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_cmd)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_odom = msg

    def stop_robot(self):
        self.pub.publish(Twist())

    def publish_result(self, value):
        msg = Bool(data=value)
        for _ in range(2):
            if rospy.is_shutdown():
                return
            self.orange_action_pub.publish(msg)
            try:
                rospy.sleep(0.1)
            except rospy.exceptions.ROSInterruptException:
                return

    def calculate_dx(self):
        current_x = self.latest_odom.pose.pose.position.x

        parts = self.orange_model.split('.')
        if len(parts) < 4:
            rospy.logwarn('Invalid orange name: %s', self.orange_model)
            self.dx = 0.0
            self.goal_x = current_x
            return

        # Example: orange2.1.R.B
        fruit_part = parts[0]   # orange2
        side_part = parts[2]    # R or L

        # get column number from orange2 -> 2
        column_id_str = fruit_part.replace('orange', '')
        if not column_id_str.isdigit():
            rospy.logwarn('Could not parse column number from: %s', self.orange_model)
            self.dx = 0.0
            self.goal_x = current_x
            return

        column_id = int(column_id_str)
        side = side_part.upper()

        # target x for each valid (column_id, side)
        target_x_map = {
            (1, 'R'): -4.5,
            (2, 'L'): -3.5,
            (2, 'R'): -0.5,
            (3, 'L'):  0.5,
            (3, 'R'):  3.5,
            (4, 'L'):  4.5,
        }

        key = (column_id, side)
        if key not in target_x_map:
            # rospy.logwarn('No target_x defined for %s', str(key))
            self.dx = 0.0
            self.goal_x = current_x
            return

        target_x = target_x_map[key]
        self.dx = target_x - current_x
        self.goal_x = target_x

    def publish_cmd(self, _event):
        if self.latest_odom is None or self.done:
            return

        if self.goal_x is None:
            self.calculate_dx()
            start_x = self.latest_odom.pose.pose.position.x
            rospy.loginfo('start_x=%.4f  goal_x=%.4f  dx=%.4f', start_x, self.goal_x, self.dx)
            return

        current_x = self.latest_odom.pose.pose.position.x
        remaining_x = self.goal_x - current_x

        if abs(remaining_x) <= self.goal_tolerance:
            self.done = True
            self.stop_robot()
            rospy.loginfo('Reached near orange')
            rospy.sleep(0.1)
            self.publish_result(True)
            rospy.signal_shutdown('orange approach complete')
            return

        world_vx = self.speed if remaining_x > 0.0 else -self.speed

        cmd = Twist()
        cmd.linear.x = math.cos(self.yaw) * world_vx
        cmd.linear.y = -math.sin(self.yaw) * world_vx
        self.pub.publish(cmd)


def main():
    rospy.init_node('approach_orange', anonymous=True)
    OrangeApproach()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
