#!/usr/bin/env python3

import os
import sys
import json
import rospy
import rospkg
import moveit_commander
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveItTargetReach(object):
    def __init__(self):
        self.marker_frame = 'world'
        self.package_name = 'rbkairos_sim_bringup'
        self.config_file_name = 'orange_metadata.json'

        self.robot_ns = rospy.get_param('~robot_ns', '/robot')
        self.orange_model = rospy.get_param('~orange', '').strip()
        self.arm_type = rospy.get_param('~arm', 'Franka')


        self.move_group_name = 'arm'
        self.ee_link = 'robot_hand_tcp'
        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_cmd_topic = '/robot/gripper/pos_traj_controller/command'
        self.gripper_open_pos = 0.038

        self.planner_id = 'OMPL'
        self.planning_time = 20.0
        self.planning_attempts = 20
        self.position_tolerance = 0.01
        self.no_solution_retries = 3
        self.move_group_wait_timeout = 30.0
        self.move_group_connect_retries = 3
        self.move_group_connect_retry_delay = 2.0

        self.target = {
            'orange_name': '',
            'diameter': 0.0,
            'object_position': [0.0, 0.0, 0.0],
            'box_position': [0.0, 0.0, 0.0],
            'box_size': [0.0, 0.0, 0.0],
        }

        self.orange_action_topic = self.robot_ns + '/orange_action'
        self.object_marker_topic = self.robot_ns + '/detected_object_marker'
        self.target_marker_topic = self.robot_ns + '/target_point_marker'

        self.object_marker_pub = rospy.Publisher(self.object_marker_topic, Marker, queue_size=1, latch=True)
        self.target_marker_pub = rospy.Publisher(self.target_marker_topic, Marker, queue_size=1, latch=True)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

    def load_target_from_config(self):
        package_path = rospkg.RosPack().get_path(self.package_name)
        config_path = os.path.join(package_path, 'config', self.config_file_name)

        with open(config_path, 'r', encoding='utf-8') as handle:
            metadata = json.load(handle)

        orange_entry = metadata['orange_models'][self.orange_model]
        variant_name = orange_entry['variant']
        variant_entry = metadata['variants'][variant_name]
        world_pose = orange_entry['world_pose']
        orange_position = variant_entry['orange_position']
        box_position = variant_entry['stem_box_position']
        diameter = float(metadata['orange_diameter'])
        box_size = [float(value) for value in metadata['stem_box']['size']]

        self.target = {
            'orange_name': self.orange_model,
            'diameter': diameter,
            'object_position': [
                float(orange_position[0]) + float(world_pose[0]),
                float(orange_position[1]) + float(world_pose[1]),
                float(orange_position[2]) + float(world_pose[2]),
            ],
            'box_position': [
                float(box_position[0]) + float(world_pose[0]),
                float(box_position[1]) + float(world_pose[1]),
                float(box_position[2]) + float(world_pose[2]),
            ],
            'box_size': box_size,
            'config_path': config_path,
        }

    def configure_robot(self):
        self.robot_ns = '/robot'
        self.move_group_name = 'arm'
        self.ee_link = 'robot_hand_tcp'
        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_cmd_topic = '/robot/gripper/pos_traj_controller/command'
        self.gripper_open_pos = 0.038

        if self.arm_type.lower() == 'ur5':
            self.move_group_name = 'ur5_arm'
            self.ee_link = 'robot_egh_gripper_grasp_link'
            self.gripper_joint = 'robot_egh_gripper_finger_left_joint'
            self.gripper_open_pos = 0.048

        if rospy.get_param('~robot_ns', '/robot').lower() == '/robot_b':
            self.robot_ns = '/robot_b'
            self.gripper_cmd_topic = '/robot_b/gripper/pos_traj_controller/command'

            if self.arm_type.lower() == 'ur5':
                self.ee_link = 'robot_b_egh_gripper_grasp_link'
                self.gripper_joint = 'robot_b_egh_gripper_finger_left_joint'
            else:
                self.ee_link = 'robot_b_hand_tcp'
                self.gripper_joint = 'robot_b_finger_joint1'

    def create_object_marker(self):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.ns = 'detected_object'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.target['object_position'][0]
        marker.pose.position.y = self.target['object_position'][1]
        marker.pose.position.z = self.target['object_position'][2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.target['diameter']
        marker.scale.y = self.target['diameter']
        marker.scale.z = self.target['diameter']

        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 0.9

        return marker

    def create_target_marker(self):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.ns = 'target_point'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.target['object_position'][0]
        marker.pose.position.y = self.target['object_position'][1]
        marker.pose.position.z = self.target['object_position'][2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def add_collision_objects(self, scene):
        orange_name = self.target['orange_name']
        object_position = self.target['object_position']

        if '.R.' in orange_name:
            top_from_bottom = (0.06760, -0.53770, 0.40465)
        else:
            top_from_bottom = (-0.06761, 0.53580, 0.40465)

        if orange_name.endswith('.T'):
            top_position = object_position
            bottom_position = [
                object_position[0] - top_from_bottom[0],
                object_position[1] - top_from_bottom[1],
                object_position[2] - top_from_bottom[2],
            ]
            paired_orange_position = bottom_position
        else:
            bottom_position = object_position
            top_position = [
                object_position[0] + top_from_bottom[0],
                object_position[1] + top_from_bottom[1],
                object_position[2] + top_from_bottom[2],
            ]
            paired_orange_position = top_position

        pose_top_support = PoseStamped()
        pose_top_support.header.frame_id = self.marker_frame
        pose_top_support.pose.position.x = top_position[0]
        pose_top_support.pose.position.y = top_position[1]
        pose_top_support.pose.position.z = top_position[2] - 0.685762
        pose_top_support.pose.orientation.w = 1.0
        scene.add_box('top_support_box', pose_top_support, size=(0.03, 0.03, 1.32421))

        pose_bottom_support = PoseStamped()
        pose_bottom_support.header.frame_id = self.marker_frame
        pose_bottom_support.pose.position.x = bottom_position[0]
        pose_bottom_support.pose.position.y = bottom_position[1]
        pose_bottom_support.pose.position.z = bottom_position[2] - 0.483475
        pose_bottom_support.pose.orientation.w = 1.0
        scene.add_box('bottom_support_box', pose_bottom_support, size=(0.03, 0.03, 0.919556))

        pose = PoseStamped()
        pose.header.frame_id = self.marker_frame
        pose.pose.position.x = self.target['object_position'][0]
        pose.pose.position.y = self.target['object_position'][1]
        pose.pose.position.z = self.target['object_position'][2]
        pose.pose.orientation.w = 1.0
        scene.add_sphere('detected_object', pose, radius=self.target['diameter'] / 2.0)

        paired_pose = PoseStamped()
        paired_pose.header.frame_id = self.marker_frame
        paired_pose.pose.position.x = paired_orange_position[0]
        paired_pose.pose.position.y = paired_orange_position[1]
        paired_pose.pose.position.z = paired_orange_position[2]
        paired_pose.pose.orientation.w = 1.0
        scene.add_sphere('paired_detected_object', paired_pose, radius=self.target['diameter'] / 2.0)

        rospy.loginfo('Collision objects for orange added to MoveIt planning scene')

    def extract_plan_result(self, plan_result):
        if isinstance(plan_result, tuple):
            if len(plan_result) >= 2:
                return bool(plan_result[0]), plan_result[1]
            return False, None

        trajectory = getattr(plan_result, 'joint_trajectory', None)
        has_points = bool(trajectory and trajectory.points)
        return has_points, plan_result

    def plan_has_trajectory(self, plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        return bool(trajectory and trajectory.points)

    def open_gripper(self, pub):
        cmd = JointTrajectory()
        cmd.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        point.positions = [self.gripper_open_pos]
        point.time_from_start = rospy.Duration(2.0)
        cmd.points = [point]

        pub.publish(cmd)

    def create_move_group_commander(self):
        wait_timeout = float(rospy.get_param('~move_group_wait_timeout', self.move_group_wait_timeout))
        retries = max(0, int(rospy.get_param('~move_group_connect_retries', self.move_group_connect_retries)))
        retry_delay = float(rospy.get_param('~move_group_connect_retry_delay', self.move_group_connect_retry_delay))

        last_error = None
        for attempt in range(1, retries + 2):
            try:
                return moveit_commander.MoveGroupCommander(
                    self.move_group_name,
                    robot_description=self.robot_ns + '/robot_description',
                    ns=self.robot_ns,
                    wait_for_servers=wait_timeout,
                )
            except RuntimeError as exc:
                last_error = exc
                rospy.logwarn(
                    'MoveGroupCommander connection failed on attempt %d/%d: %s',
                    attempt,
                    retries + 1,
                    str(exc),
                )
                if attempt <= retries and not rospy.is_shutdown():
                    rospy.sleep(retry_delay)

        raise last_error

    def move_arm_to_target(self):
        no_solution_retries = max(0, int(rospy.get_param('~no_solution_retries', self.no_solution_retries)))

        moveit_commander.roscpp_initialize(sys.argv)
        scene = PlanningSceneInterface(ns=self.robot_ns)
        group = self.create_move_group_commander()
        gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)

        group.set_planner_id(self.planner_id)
        group.set_planning_time(self.planning_time)
        group.set_num_planning_attempts(self.planning_attempts)
        group.set_goal_position_tolerance(self.position_tolerance)

        resolved_ee_link = self.ee_link if self.ee_link else group.get_end_effector_link()
        rospy.loginfo(
            'Using end-effector: %s | Target: [%.5f, %.5f, %.5f]',
            resolved_ee_link,
            self.target['object_position'][0],
            self.target['object_position'][1],
            self.target['object_position'][2],
        )

        rospy.sleep(1.0)
        self.add_collision_objects(scene)
        rospy.sleep(5.0)

        group.set_pose_reference_frame(self.marker_frame)
        group.set_position_target(self.target['object_position'], resolved_ee_link)

        plan = None
        for attempt in range(1, no_solution_retries + 2):
            rospy.loginfo(
                'Planning to target (attempt %d/%d)',
                attempt,
                no_solution_retries + 1,
            )

            planned, candidate_plan = self.extract_plan_result(group.plan())
            if planned and self.plan_has_trajectory(candidate_plan):
                plan = candidate_plan
                break

            if attempt <= no_solution_retries:
                rospy.logwarn('MoveIt found no plan; opening gripper before replanning')
                self.open_gripper(gripper_pub)
                rospy.sleep(2.0)

        if plan is None:
            success = False
            rospy.logerr('MoveIt found no valid plan after retries; not executing')
        else:
            success = group.execute(plan, wait=True)
            group.stop()

        group.clear_pose_targets()
        rospy.loginfo('Arm reached the target position')
        moveit_commander.roscpp_shutdown()

        return success

    def run(self):
        if not self.orange_model:
            rospy.logerr('Set ~orange to an orange model name, for example orange2.4.R.T')
            return

        try:
            self.load_target_from_config()
        except Exception as exc:
            rospy.logerr('Failed to load orange metadata for %s: %s', self.orange_model, str(exc))
            return

        self.configure_robot()

        object_marker = self.create_object_marker()
        target_marker = self.create_target_marker()

        moved = False
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            object_marker.header.stamp = rospy.Time.now()
            target_marker.header.stamp = rospy.Time.now()

            self.object_marker_pub.publish(object_marker)
            self.target_marker_pub.publish(target_marker)

            if not moved:
                moved = True
                success = self.move_arm_to_target()
                self.orange_action_pub.publish(Bool(data=True))
                return

            rate.sleep()


def main():
    rospy.init_node('moveit_target_reach', anonymous=True)
    MoveItTargetReach().run()


if __name__ == '__main__':
    main()