#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


class OrangePoseToWorld(object):
    def __init__(self):
        self.robot_ns = rospy.get_param('~robot_ns', '/robot')
        self.input_topic = rospy.get_param(
            '~input_topic',
            self.robot_ns + '/object_position'
        )
        self.output_topic = rospy.get_param(
            '~output_topic',
            self.robot_ns + '/detected_orange_pose_world'
        )
        self.target_frame = rospy.get_param('~target_frame', 'world')

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=1)
        rospy.Subscriber(self.input_topic, PointStamped, self.pose_cb, queue_size=1)

    def pose_cb(self, msg):
        try:
            world_pose = self.tf_buffer.transform(msg, self.target_frame, rospy.Duration(1.0))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, 'Orange pose TF failed: %s', str(exc))
            return

        self.pub.publish(world_pose)
        rospy.loginfo_throttle(
            1.0,
            'Published orange world pose: [%.3f, %.3f, %.3f]',
            world_pose.point.x,
            world_pose.point.y,
            world_pose.point.z,
        )

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('orange_pose_to_world')
    OrangePoseToWorld().run()


if __name__ == '__main__':
    main()
