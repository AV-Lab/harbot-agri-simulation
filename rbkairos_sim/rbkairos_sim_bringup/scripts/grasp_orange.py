#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Bool
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class OrangeGrasper(object):
    def __init__(self):
        self.robot_ns = rospy.get_param('~robot_ns', '/robot_b')
        self.orange_model = rospy.get_param('~orange', '').strip()
        self.arm_type = rospy.get_param('~arm', 'Franka').lower()
        self.grasp_success_probability = float(rospy.get_param('~grasp_success_probability', 1.00)) 
        self.orange_link = 'body'

        self.robot_model = 'robot'
        self.robot_base_link = 'robot_base_footprint'
        self.gripper_link = 'robot_link7'
        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_close_pos = 0.2

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
        self.pickup_bottom_pos = [-0.429, 0.233, 0.030, -1.904, 1.325, 1.211, -1.360]
        self.pickup_top_pos = [-0.249, -0.914, -0.461, -2.035, 1.494, 0.906, -0.273]

        self.arm_cmd_topic = self.robot_ns + '/arm/pos_traj_controller/command'
        self.gripper_cmd_topic = self.robot_ns + '/gripper/pos_traj_controller/command'
        self.orange_action_topic = self.robot_ns + '/orange_action'

        self.configure_robot()

        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

    def set_link_attachment(self, action, model_name_1, link_name_1, model_name_2, link_name_2):
        service_name = "/link_attacher_node/{}".format(action)

        try:
            rospy.wait_for_service(service_name, timeout=0.5)
        except rospy.ROSException:
            rospy.logwarn("%s service is not available", service_name)
            return False

        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2

        try:
            service = rospy.ServiceProxy(service_name, Attach)
            result = service(req)
            rospy.sleep(0.5)
            return result.ok
        except rospy.ServiceException as exc:
            rospy.logwarn("%s service failed: %s", service_name, str(exc))
            return False

    def simulated_grasp_succeeds(self):
        if self.grasp_success_probability < 0.0 or self.grasp_success_probability > 1.0:
            rospy.logwarn('Invalid grasp_success_probability. Using 0.8')
            self.grasp_success_probability = 0.8

        return random.random() < self.grasp_success_probability

    def configure_robot(self):
        if self.robot_ns == '/robot_b':
            self.robot_model = 'robot_b'
            self.robot_base_link = 'robot_b_base_footprint'

        if self.arm_type == 'franka':
            prefix = 'robot_b' if self.robot_ns == '/robot_b' else 'robot'

            self.arm_joints = [
                prefix + '_joint1',
                prefix + '_joint2',
                prefix + '_joint3',
                prefix + '_joint4',
                prefix + '_joint5',
                prefix + '_joint6',
                prefix + '_joint7',
            ]

            self.gripper_link = prefix + '_link7'
            self.gripper_joint = prefix + '_finger_joint1'
            self.gripper_close_pos = 0.2

        elif self.arm_type == 'ur5':
            prefix = 'robot_b' if self.robot_ns == '/robot_b' else 'robot'

            self.arm_joints = [
                prefix + '_arm_shoulder_pan_joint',
                prefix + '_arm_shoulder_lift_joint',
                prefix + '_arm_elbow_joint',
                prefix + '_arm_wrist_1_joint',
                prefix + '_arm_wrist_2_joint',
                prefix + '_arm_wrist_3_joint',
            ]

            self.arm_initial_pos = [-0.775, -0.784, -2.000, -0.009, -0.011, -0.000]
            self.gripper_link = prefix + '_arm_wrist_3_link'
            self.gripper_joint = prefix + '_egh_gripper_finger_left_joint'
            self.gripper_close_pos = 0.032

    def build_joint_command(self, joint_names, positions, duration):
        cmd = JointTrajectory()
        cmd.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)

        cmd.points = [point]
        return cmd

    def close_gripper(self):
        cmd = self.build_joint_command(
            [self.gripper_joint],
            [self.gripper_close_pos],
            2.0
        )
        self.gripper_pub.publish(cmd)

    def move_arm(self, positions, duration=3.0):
        cmd = self.build_joint_command(
            self.arm_joints,
            positions,
            duration
        )
        self.arm_pub.publish(cmd)

    def run(self):
        try:
            if not self.simulated_grasp_succeeds():
                rospy.logwarn(
                    "Grasp failed for %s (success probability %.2f)",
                    self.orange_model,
                    self.grasp_success_probability,
                )
                rospy.sleep(3.0)
                self.move_arm(self.arm_initial_pos)
                rospy.sleep(5.0)
                if not self.set_link_attachment(
                    'detach',
                    self.robot_model,
                    self.robot_base_link,
                    self.orange_model,
                    self.orange_link,
                ):
                    rospy.logwarn('Base detach failed for %s', self.orange_model)
                else:
                    rospy.loginfo("DEATTACHED")
                self.orange_action_pub.publish(Bool(data=False))
                return                
            
            rospy.sleep(0.5)
            
            if not self.set_link_attachment(
                'detach',
                self.robot_model,
                self.robot_base_link,
                self.orange_model,
                self.orange_link,
            ):
                rospy.logwarn('Base detach failed for %s', self.orange_model)
                self.orange_action_pub.publish(Bool(data=False))
                return
            else:
                rospy.loginfo('Detached orange from robot base')

            rospy.sleep(0.5)

            if not self.set_link_attachment(
                'attach',
                self.robot_model,
                self.gripper_link,
                self.orange_model,
                self.orange_link,
            ):
                rospy.logwarn('Attach failed')
                self.orange_action_pub.publish(Bool(data=False))
                return

            rospy.loginfo('Grasping orange')
            rospy.sleep(2.0)

            self.close_gripper()
            rospy.sleep(2.0)

            rospy.loginfo('Grasped orange. Moving to unload position')
            self.move_arm(self.arm_initial_pos)
            rospy.sleep(3.0)

            rospy.loginfo('Arm moved to holding position')
            self.orange_action_pub.publish(Bool(data=True))

        except rospy.ServiceException as exc:
            rospy.logerr('Attach service failed: %s', str(exc))
            self.orange_action_pub.publish(Bool(data=False))


def main():
    rospy.init_node('grasp_orange', anonymous=True)
    OrangeGrasper().run()


if __name__ == '__main__':
    main()