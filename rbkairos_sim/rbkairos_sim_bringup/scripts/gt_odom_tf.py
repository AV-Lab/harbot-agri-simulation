#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from tf import transformations as tft

latest_world_base = None
br = None
listener = None
gt_odom_topic = None
world_frame = None
odom_frame = None
base_frame = None


def pose_to_matrix(position, orientation):
    T = tft.quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    T[0, 3] = position.x
    T[1, 3] = position.y
    T[2, 3] = position.z
    return T


def odom_gt_callback(msg):
    global latest_world_base
    latest_world_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
    rospy.loginfo_once("Ground truth odometry received")


def broadcast_odom_tf():
    global latest_world_base, br, listener, world_frame, odom_frame, base_frame
    
    if latest_world_base is None:
        return

    try:
        trans, rot = listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        T_odom_base = tft.concatenate_matrices(
            tft.translation_matrix(trans),
            tft.quaternion_matrix(rot),
        )
        T_world_odom = tft.concatenate_matrices(
            latest_world_base,
            tft.inverse_matrix(T_odom_base),
        )
        world_odom_trans = tft.translation_from_matrix(T_world_odom)
        world_odom_rot = tft.quaternion_from_matrix(T_world_odom)

        br.sendTransform(
            world_odom_trans,
            world_odom_rot,
            rospy.Time.now(),
            odom_frame,
            world_frame,
        )
    except (
        tf.LookupException,
        tf.ConnectivityException,
        tf.ExtrapolationException,
    ) as exc:
        rospy.logwarn("TF lookup failed: %s", str(exc))


def main():
    global br, listener, gt_odom_topic, world_frame, odom_frame, base_frame
    
    rospy.init_node("gt_odom_tf")
    gt_odom_topic = rospy.get_param("~gt_odom_topic", "/robot/robotnik_base_control/odom_gt")
    world_frame = rospy.get_param("~world_frame", "world")
    odom_frame = rospy.get_param("~odom_frame", "robot_odom")
    base_frame = rospy.get_param("~base_frame", "robot_base_footprint")
    rate_hz = rospy.get_param("~rate", 30.0)

    rospy.Subscriber(gt_odom_topic, Odometry, odom_gt_callback, queue_size=1)
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    
    rate = rospy.Rate(rate_hz)
    rospy.loginfo_once("Ground truth odometry TF broadcaster started")
    
    while not rospy.is_shutdown():
        broadcast_odom_tf()
        
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    main()