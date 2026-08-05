#!/usr/bin/env python3
import math

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointStateGuiToController:
    def __init__(self):
        self.joints = rospy.get_param(
            "~joints",
            [
                "robot_joint1",
                "robot_joint2",
                "robot_joint3",
                "robot_joint4",
                "robot_joint5",
                "robot_joint6",
                "robot_joint7",
            ],
        )
        self.duration = rospy.Duration.from_sec(rospy.get_param("~trajectory_duration", 0.6))
        self.deadband = float(rospy.get_param("~deadband", 0.003))
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance", 0.08))
        self.min_publish_period = rospy.Duration.from_sec(
            rospy.get_param("~min_publish_period", 0.08)
        )

        input_topic = rospy.get_param("~input_joint_states", "franka_arm_gui_joint_states")
        feedback_topic = rospy.get_param("~feedback_joint_states", "joint_states")
        command_topic = rospy.get_param("~command_topic", "arm/pos_traj_controller/command")

        self.feedback_positions = None
        self.last_command = None
        self.armed = False
        self.last_publish_time = rospy.Time(0)

        self.command_pub = rospy.Publisher(command_topic, JointTrajectory, queue_size=1)
        rospy.Subscriber(feedback_topic, JointState, self.feedback_cb, queue_size=1)
        rospy.Subscriber(input_topic, JointState, self.gui_cb, queue_size=1)

        rospy.loginfo(
            "Bridge ready: %s -> %s, feedback %s, joints %s",
            input_topic,
            command_topic,
            feedback_topic,
            ", ".join(self.joints),
        )

    def positions_for_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(joint in values for joint in self.joints):
            return None

        positions = [values[joint] for joint in self.joints]
        if not all(math.isfinite(position) for position in positions):
            return None

        return positions

    def feedback_cb(self, msg):
        positions = self.positions_for_joints(msg)
        if positions is not None:
            self.feedback_positions = positions

    @staticmethod
    def max_delta(a, b):
        return max(abs(left - right) for left, right in zip(a, b))

    def gui_cb(self, msg):
        target = self.positions_for_joints(msg)
        if target is None:
            return

        if self.feedback_positions is None:
            rospy.logwarn_throttle(2.0, "Waiting for live arm feedback before sending commands.")
            return

        if not self.armed:
            if self.max_delta(target, self.feedback_positions) > self.sync_tolerance:
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting for GUI sliders to start near live /robot/joint_states before commanding.",
                )
                return

            self.armed = True
            self.last_command = list(self.feedback_positions)
            rospy.loginfo("GUI sliders synced with live arm feedback; controller bridge armed.")
            return

        if self.max_delta(target, self.last_command) < self.deadband:
            return

        now = rospy.Time.now()
        if now - self.last_publish_time < self.min_publish_period:
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = now
        trajectory.joint_names = list(self.joints)

        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(self.joints)
        point.time_from_start = self.duration
        trajectory.points.append(point)

        self.command_pub.publish(trajectory)
        self.last_command = list(target)
        self.last_publish_time = now


if __name__ == "__main__":
    rospy.init_node("franka_arm_gui_to_controller")
    JointStateGuiToController()
    rospy.spin()
