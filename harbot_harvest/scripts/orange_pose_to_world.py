#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Receive the detected orange position in the camera frame.
# - Transform it into the target frame (world by default) with TF.
# - Publish the transformed position for the MoveIt reach step.
# ******************************************************************************


import rospy
import tf2_ros
# Registers PointStamped with tf2, so tf_buffer.transform() accepts it.
import tf2_geometry_msgs  # noqa: F401
from harbot_harvest import helpers
from geometry_msgs.msg import PointStamped


class OrangePoseToWorld:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.target_frame = rospy.get_param("~target_frame", "world")
        self.input_topic = rospy.get_param("~input_topic", self.robot_ns + "/object_position")
        self.output_topic = rospy.get_param("~output_topic", self.robot_ns + "/detected_orange_pose_world")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=1)

        rospy.Subscriber(self.input_topic, PointStamped, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        try:
            world_pose = self.tf_buffer.transform(msg, self.target_frame, rospy.Duration(1.0))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Orange pose TF failed: %s", exc)
            return

        self.pose_pub.publish(world_pose)


def main():
    rospy.init_node("orange_pose_to_world", anonymous=True)
    OrangePoseToWorld()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
