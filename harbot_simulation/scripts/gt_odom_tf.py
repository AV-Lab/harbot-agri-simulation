#!/usr/bin/env python3

import tf
import rospy
from nav_msgs.msg import Odometry
from tf import transformations as tft


class GroundTruthOdomBroadcaster(object):
    def __init__(self):
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.odom_frame = rospy.get_param('~odom_frame', 'robot_odom')
        self.base_frame = rospy.get_param('~base_frame', 'robot_base_footprint')
        self.gt_odom_topic = rospy.get_param('~gt_odom_topic', '/robot/robotnik_base_control/odom_gt')

        self.rate_hz = rospy.get_param('~rate', 30.0)

        self.latest_world_base = None

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        rospy.Subscriber(self.gt_odom_topic, Odometry, self.odom_gt_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.broadcast_odom_tf)

        rospy.loginfo_once('Ground truth odometry TF broadcaster started')

    def pose_to_matrix(self, position, orientation):
        transform = tft.quaternion_matrix([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])

        transform[0, 3] = position.x
        transform[1, 3] = position.y
        transform[2, 3] = position.z

        return transform

    def odom_gt_callback(self, msg):
        self.latest_world_base = self.pose_to_matrix(
            msg.pose.pose.position,
            msg.pose.pose.orientation
        )
        rospy.loginfo_once('Ground truth odometry received')

    def broadcast_odom_tf(self, _event):
        if self.latest_world_base is None:
            return

        try:
            trans, rot = self.listener.lookupTransform(
                self.odom_frame,
                self.base_frame,
                rospy.Time(0)
            )

            odom_base = tft.concatenate_matrices(
                tft.translation_matrix(trans),
                tft.quaternion_matrix(rot),
            )

            world_odom = tft.concatenate_matrices(
                self.latest_world_base,
                tft.inverse_matrix(odom_base),
            )

            world_odom_trans = tft.translation_from_matrix(world_odom)
            world_odom_rot = tft.quaternion_from_matrix(world_odom)

            self.br.sendTransform(
                world_odom_trans,
                world_odom_rot,
                rospy.Time.now(),
                self.odom_frame,
                self.world_frame,
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn('TF lookup failed: %s', str(exc))


def main():
    rospy.init_node('gt_odom_tf', anonymous=True)
    GroundTruthOdomBroadcaster()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()