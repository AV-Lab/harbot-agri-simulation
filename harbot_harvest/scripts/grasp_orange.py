#!/usr/bin/env python3

import sys
import rospy
import random
import moveit_commander
from std_msgs.msg import Bool
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class OrangeGrasper(object):
    def __init__(self):
        self.robot_ns = rospy.get_param('~robot_ns', '/robot_b')
        self.orange_model = rospy.get_param('~orange', '').strip()
        self.arm_type = rospy.get_param('~arm', 'Franka').lower()
        self.grasp_success_probability = float(rospy.get_param('~grasp_success_probability', 0.91)) # prob from real-world experiments  
        self.orange_link = 'body'

        self.robot_model = 'robot'
        self.robot_base_link = 'robot_base_footprint'
        self.gripper_link = 'robot_link7'
        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_close_pos = -0.1
        self.move_group_name = 'arm'

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

        self.arm_cmd_topic = self.robot_ns + '/arm/pos_traj_controller/command'
        self.gripper_cmd_topic = self.robot_ns + '/gripper/pos_traj_controller/command'
        self.orange_action_topic = self.robot_ns + '/orange_action'
        self.return_planning_time = float(rospy.get_param('~return_planning_time', 10.0))
        self.return_planning_attempts = int(rospy.get_param('~return_planning_attempts', 10))
        self.return_motion_retries = int(rospy.get_param('~return_motion_retries', -1))
        self.return_motion_retry_delay = float(rospy.get_param('~return_motion_retry_delay', 0.5))
        self.move_group_wait_timeout = float(rospy.get_param('~move_group_wait_timeout', 50.0))

        self.configure_robot()

        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

    def set_link_attachment(
        self,
        action,
        model_name_1,
        link_name_1,
        model_name_2,
        link_name_2,
        attempts=3,
    ):
        service_name = "/link_attacher_node/{}".format(action)

        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2

        for attempt in range(1, attempts + 1):
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
                service = rospy.ServiceProxy(service_name, Attach)
                result = service(req)
                rospy.sleep(0.1)
                if result.ok:
                    return True

                rospy.logwarn(
                    "%s returned ok=False on attempt %d/%d: %s/%s -> %s/%s",
                    service_name,
                    attempt,
                    attempts,
                    model_name_1,
                    link_name_1,
                    model_name_2,
                    link_name_2,
                )
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn(
                    "%s failed on attempt %d/%d: %s",
                    service_name,
                    attempt,
                    attempts,
                    str(exc),
                )

            if attempt < attempts:
                rospy.sleep(0.1)

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
            self.gripper_close_pos = -0.1

        elif self.arm_type == 'ur5':
            prefix = 'robot_b' if self.robot_ns == '/robot_b' else 'robot'
            self.move_group_name = 'ur5_arm'

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

    def move_arm_to_initial_position_with_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        group = None
        attempt = 1

        try:
            while not rospy.is_shutdown():
                try:
                    if group is None:
                        group = moveit_commander.MoveGroupCommander(
                            self.move_group_name,
                            robot_description=self.robot_ns + '/robot_description',
                            ns=self.robot_ns,
                            wait_for_servers=self.move_group_wait_timeout,
                        )
                        group.set_planning_time(self.return_planning_time)
                        group.set_num_planning_attempts(self.return_planning_attempts)
                        group.set_max_acceleration_scaling_factor(0.6)
                        group.set_max_velocity_scaling_factor(0.4)

                    group.set_start_state_to_current_state()
                    group.set_joint_value_target(dict(zip(self.arm_joints, self.arm_initial_pos)))

                    ok = group.go(wait=True)
                    group.stop()
                    group.clear_pose_targets()
                    if ok:
                        return True

                    # rospy.logwarn('MoveIt initial-position motion attempt %d failed; retrying', attempt)
                except Exception as exc:
                    # rospy.logwarn('MoveIt initial-position motion attempt %d failed: %s', attempt, str(exc))
                    if group is not None:
                        try:
                            group.stop()
                            group.clear_pose_targets()
                        except Exception:
                            pass
                    group = None

                if self.return_motion_retries >= 0 and attempt >= self.return_motion_retries:
                    return False

                attempt += 1
                rospy.sleep(self.return_motion_retry_delay)

            return False
        finally:
            moveit_commander.roscpp_shutdown()

    def detach_orange_from_base(self):
        # rospy.loginfo('Detaching %s from robot base', self.orange_model)
        ok = self.set_link_attachment(
            'detach',
            self.robot_model,
            self.robot_base_link,
            self.orange_model,
            self.orange_link,
            attempts=5,
        )

        if ok:
            # rospy.loginfo('Detached %s from robot base', self.orange_model)
            rospy.sleep(0.5)
            return True

        # rospy.logwarn('Could not detach %s from robot base', self.orange_model)
        return False

    def run(self):
        try:
            if not self.simulated_grasp_succeeds():
                rospy.logwarn(
                    'Grasp attempt failed for %s (simulated success probability %.2f).',
                    self.orange_model,
                    self.grasp_success_probability,
                )
                if not self.move_arm_to_initial_position_with_moveit():
                    self.orange_action_pub.publish(Bool(data=False))
                    return
                rospy.sleep(1.0)
                self.detach_orange_from_base()
                self.orange_action_pub.publish(Bool(data=False))
                return                
            
            rospy.sleep(0.1)
            
            if not self.set_link_attachment(
                'attach',
                self.robot_model,
                self.gripper_link,
                self.orange_model,
                self.orange_link,
            ):
                # rospy.logwarn('Attach failed')
                self.orange_action_pub.publish(Bool(data=False))
                return

            rospy.sleep(0.5)

            if not self.detach_orange_from_base():
                self.set_link_attachment(
                    'detach',
                    self.robot_model,
                    self.gripper_link,
                    self.orange_model,
                    self.orange_link,
                )
                self.orange_action_pub.publish(Bool(data=False))
                return

            rospy.loginfo('Grasping orange')

            self.close_gripper()
            rospy.sleep(2.0)

            rospy.loginfo('Grasped orange. Moving to unload position using MoveIt')
            if not self.move_arm_to_initial_position_with_moveit():
                self.orange_action_pub.publish(Bool(data=False))
                return

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
