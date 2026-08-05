#!/usr/bin/env python3

import argparse

import rospy
from geometry_msgs.msg import PointStamped


DEFAULT_TOPIC = '/object_position'
DEFAULT_FRAME = 'robot_wrist_d435_color_optical_frame'


def parse_args():
    parser = argparse.ArgumentParser(description='Publish a fake orange pose.')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('--frame', default=DEFAULT_FRAME)
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--rate', type=float, default=1.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node('fake_orange_pose_pub')
    pub = rospy.Publisher(args.topic, PointStamped, queue_size=1)
    rate = rospy.Rate(args.rate)

    while not rospy.is_shutdown():
        pose = PointStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = args.frame
        pose.point.x = args.x
        pose.point.y = args.y
        pose.point.z = args.z

        pub.publish(pose)
        rospy.loginfo_throttle(
            1.0,
            'Fake orange pose: %s [%.3f, %.3f, %.3f]',
            pose.header.frame_id,
            pose.point.x,
            pose.point.y,
            pose.point.z,
        )
        rate.sleep()


if __name__ == '__main__':
    main()
