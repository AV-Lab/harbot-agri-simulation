#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from harbot_harvest.attachment import (
    ATTACHMENT_TARGET_RELEASED,
    ObjectAttachmentClient,
)
from harbot_harvest.helpers import resolve_ros_name


class OrangeUnloader(object):
    def __init__(self):
        self.orange_model = rospy.get_param('~orange', '')
        self.robot_ns = rospy.get_param('~robot_ns', '/robot')
        self.attachment_client = ObjectAttachmentClient(self.robot_ns)

        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_cmd_topic = resolve_ros_name(
            self.robot_ns,
            rospy.get_param('~gripper_command_topic', 'gripper/pos_traj_controller/command'),
        )

        self.arm_cmd_topic = resolve_ros_name(
            self.robot_ns,
            rospy.get_param('~arm_command_topic', 'arm/pos_traj_controller/command'),
        )
        self.arm_joints = [
            'robot_joint1',
            'robot_joint2',
            'robot_joint3',
            'robot_joint4',
            'robot_joint5',
            'robot_joint6',
            'robot_joint7',
        ]

        self.arm_initial_pos = [0.000, -0.784, 0.000, -2.100, 0.000, 1.569, 0.782]
        self.arm_drop_pos = [0.001, 0.615, -0.000, -1.754, 0.000, 2.463, 0.785]
        self.orange_action_topic = resolve_ros_name(
            self.robot_ns,
            rospy.get_param('~result_topic', 'orange_action'),
        )

        self.configure_robot()

        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)
        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

    def configure_robot(self):
        if self.robot_ns.lower() == '/robot_b':
            self.arm_joints = [
                'robot_b_joint1',
                'robot_b_joint2',
                'robot_b_joint3',
                'robot_b_joint4',
                'robot_b_joint5',
                'robot_b_joint6',
                'robot_b_joint7',
            ]

            self.gripper_joint = 'robot_b_finger_joint1'

    def build_joint_command(self, joint_names, positions, duration):
        cmd = JointTrajectory()
        cmd.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)

        cmd.points = [point]
        return cmd

    def open_gripper(self):
        cmd = self.build_joint_command(
            [self.gripper_joint],
            [0.5],
            2.0
        )
        self.gripper_pub.publish(cmd)

    def move_arm_to_initial_position(self):
        cmd = self.build_joint_command(
            self.arm_joints,
            self.arm_initial_pos,
            3.0
        )
        self.arm_pub.publish(cmd)

    def move_arm_to_drop_position(self):
        cmd = self.build_joint_command(
            self.arm_joints,
            self.arm_drop_pos,
            3.0
        )
        self.arm_pub.publish(cmd)

    def run(self):
        if not self.orange_model:
            rospy.logerr('Parameter ~orange is required')
            return

        rospy.sleep(0.5)
        
        rospy.loginfo('Moving orange to drop position')
        self.move_arm_to_drop_position()
        rospy.sleep(4.0)

        if not self.attachment_client.set_target(
            self.orange_model,
            ATTACHMENT_TARGET_RELEASED,
        ):
            rospy.logerr('Could not release %s through the attachment service', self.orange_model)
            self.orange_action_pub.publish(Bool(data=False))
            return
        rospy.sleep(0.1)

        rospy.loginfo('Unloading orange')
        self.open_gripper()

        rospy.sleep(0.5)

        rospy.loginfo('Unloaded orange. Returning to initial position')
        self.move_arm_to_initial_position()

        rospy.sleep(0.5)

        rospy.loginfo('Arm returned to initial position')
        self.orange_action_pub.publish(Bool(data=True))


def main():
    rospy.init_node('unload_orange', anonymous=True)
    OrangeUnloader().run()


if __name__ == '__main__':
    main()
