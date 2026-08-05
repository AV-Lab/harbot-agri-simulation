#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from gazebo_msgs.srv import GetModelProperties
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class OrangeUnloader(object):
    def __init__(self):
        self.orange_model = rospy.get_param('~orange', '')
        self.robot_ns = rospy.get_param('~robot_ns', '/robot')

        self.robot_model = 'robot'
        self.robot_link = 'robot_link7'

        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_cmd_topic = '/robot/gripper/pos_traj_controller/command'

        self.arm_cmd_topic = '/robot/arm/pos_traj_controller/command'
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
        self.orange_action_topic = self.robot_ns + '/orange_action'

        self.configure_robot()

        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)
        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

    def configure_robot(self):
        if self.robot_ns.lower() == '/robot_b':
            self.robot_model = 'robot_b'
            self.robot_link = 'robot_b_link7'

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
            self.gripper_cmd_topic = '/robot_b/gripper/pos_traj_controller/command'
            self.arm_cmd_topic = '/robot_b/arm/pos_traj_controller/command'

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
            return result.ok
        except rospy.ServiceException as exc:
            rospy.logwarn("%s service failed: %s", service_name, str(exc))
            return False

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
            [0.25],
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

    def get_model_link_name(self):
        rospy.wait_for_service('/gazebo/get_model_properties')
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        response = get_model_properties(self.orange_model)

        if response.success and response.canonical_body_name:
            return response.canonical_body_name.split('::')[-1]

        return 'body'

    def run(self):
        if not self.orange_model:
            rospy.logerr('Parameter ~orange is required')
            return

        model_link = self.get_model_link_name()

        rospy.sleep(1.0)
        
        rospy.loginfo('Moving orange to drop position')
        self.move_arm_to_drop_position()
        rospy.sleep(5.0)

        self.set_link_attachment(
            'detach',
            self.robot_model,
            self.robot_link,
            self.orange_model,
            model_link,
        )
        rospy.sleep(1.0)

        rospy.loginfo('Unloading orange')
        self.open_gripper()

        rospy.sleep(1.0)

        rospy.loginfo('Unloaded orange. Returning to initial position')
        self.move_arm_to_initial_position()

        rospy.sleep(3.0)

        rospy.loginfo('Arm returned to initial position')
        self.orange_action_pub.publish(Bool(data=True))


def main():
    rospy.init_node('unload_orange', anonymous=True)
    OrangeUnloader().run()


if __name__ == '__main__':
    main()