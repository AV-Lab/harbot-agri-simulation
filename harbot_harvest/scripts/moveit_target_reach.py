#!/usr/bin/env python3

import os
import sys
import json
import signal
import time
import rospy
import rospkg
import subprocess
import moveit_commander
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_commander import PlanningSceneInterface
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveItTargetReach(object):
    def __init__(self):
        self.marker_frame = 'world'
        self.package_name = 'harbot_harvest'

        self.robot_ns = rospy.get_param('~robot_ns', '/robot')
        self.detected_pose_topic = rospy.get_param(
            '~detected_pose_topic',
            self.robot_ns + '/detected_orange_pose_world'
        )
        self.launch_detection = bool(rospy.get_param('~launch_detection', True))
        self.detection_startup_delay = float(rospy.get_param('~detection_startup_delay', 0.1))
        self.detection_timeout = float(rospy.get_param('~detection_timeout', 10.0))
        self.orange_model = rospy.get_param('~orange', '').strip()
        self.robot_model = 'robot'
        self.robot_base_link = 'robot_base_footprint'
        self.arm_type = rospy.get_param('~arm', 'Franka')


        self.move_group_name = 'arm'
        self.ee_link = 'robot_hand_tcp'
        self.gripper_joint = 'robot_finger_joint1'
        self.gripper_cmd_topic = '/robot/gripper/pos_traj_controller/command'
        self.gripper_open_pos = 0.5
        self.arm_cmd_topic = self.robot_ns + '/arm/pos_traj_controller/command'
        self.arm_joints = [
            'robot_joint1',
            'robot_joint2',
            'robot_joint3',
            'robot_joint4',
            'robot_joint5',
            'robot_joint6',
            'robot_joint7',
        ]
        self.arm_pub = None

        self.diameter = 0.07
        self.planner_id = 'RRTConnect'
        self.latest_odom = None
        self.planning_time = 40.0
        self.planning_attempts = 14
        self.position_tolerance = 0.01
        self.no_solution_retries = 14
        self.move_group_wait_timeout = 50.0
        self.move_group_connect_retries = 9
        self.move_group_connect_retry_delay = 2.0
        self.moveit_planning_time_sec = 0.0

        self.target = {
            'orange_name': self.orange_model,
            'diameter': self.diameter,
            'object_position': None,
            'box_position': [0.0, 0.0, 0.0],
            'box_size': [0.0, 0.0, 0.0],
        }

        self.orange_action_topic = self.robot_ns + '/orange_action'
        self.object_marker_topic = self.robot_ns + '/detected_object_marker'
        self.target_marker_topic = self.robot_ns + '/target_point_marker'
        self.selected_collision_marker_topic = self.robot_ns + '/selected_collision_markers'
        self.odom_topic = self.robot_ns + '/robotnik_base_control/odom_gt'

        self.object_marker_pub = rospy.Publisher(self.object_marker_topic, Marker, queue_size=1, latch=True)
        self.target_marker_pub = rospy.Publisher(self.target_marker_topic, Marker, queue_size=1, latch=True)
        self.selected_collision_marker_pub = rospy.Publisher(self.selected_collision_marker_topic, MarkerArray, queue_size=1, latch=True)
        self.orange_action_pub = rospy.Publisher(self.orange_action_topic, Bool, queue_size=1)

        self.moveit_planning_time_pub = rospy.Publisher(self.robot_ns + '/moveit_planning_time_sec', Float64, queue_size=1, latch=True)
        self.moveit_planning_done_time_pub = rospy.Publisher(self.robot_ns + '/moveit_planning_done_time', Float64, queue_size=1, latch=True)
        
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

    
    def odom_cb(self, msg):
        self.latest_odom = msg

    def detected_x_direction(self, detected_x):
        if self.latest_odom is None:
            raise RuntimeError('No odom received yet')

        robot_x = self.latest_odom.pose.pose.position.x
        diff = detected_x - robot_x

        if diff > 0:
            return 1
        elif diff < 0:
            return -1
        return 0
    
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
            rospy.sleep(0.1)
            result = service(req)
            return result.ok
        except rospy.ServiceException as exc:
            rospy.logwarn("%s service failed: %s", service_name, str(exc))
            return False

    def launch_detection_processes(self):
        if not self.launch_detection:
            return []

        processes = []
        scripts = [
            # ("hsv_object_detector.py", {"robot_ns": self.robot_ns, "color_name": "orange"}),
            ("orange_pose_to_world.py", {"robot_ns": self.robot_ns}),
        ]

        for script_name, params in scripts:
            command = ["rosrun", self.package_name, script_name]
            command.extend("_{}:={}".format(key, value) for key, value in params.items())
            # rospy.loginfo("Starting %s for %s", script_name, self.robot_ns)
            processes.append(subprocess.Popen(command, preexec_fn=os.setsid))

        rospy.sleep(self.detection_startup_delay)
        return processes

    def shutdown_detection_processes(self, processes):
        for process_handle in processes:
            if process_handle.poll() is not None:
                continue

            try:
                os.killpg(os.getpgid(process_handle.pid), signal.SIGINT)
                process_handle.wait(timeout=3.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(process_handle.pid), signal.SIGTERM)
                except OSError:
                    pass


    def load_orange_detection(self):
        rospy.loginfo('Waiting for detected orange pose on %s', self.detected_pose_topic)

        try:
            msg = rospy.wait_for_message(
                self.detected_pose_topic,
                PointStamped,
                timeout=self.detection_timeout
            )
        except rospy.ROSException as exc:
            raise RuntimeError(
                'No detected orange pose received on {} within {:.1f} seconds'.format(
                    self.detected_pose_topic,
                    self.detection_timeout,
                )
            ) from exc

        if msg.header.frame_id != self.marker_frame:
            raise RuntimeError(
                'Expected detected orange pose in frame {}, got {}'.format(
                    self.marker_frame,
                    msg.header.frame_id,
                )
            )
        offset_direction = self.detected_x_direction(msg.point.x)
        
        self.target['object_position'] = [
            msg.point.x + (self.diameter / 2.0) * offset_direction,
            msg.point.y,
            msg.point.z,
        ]

        if self.target['object_position'] is None:
            raise RuntimeError('No detected orange position available')

        rospy.loginfo(
            'Detected MoveIt target: [%.5f, %.5f, %.5f]',
            self.target['object_position'][0],
            self.target['object_position'][1],
            self.target['object_position'][2],
        )

    def configure_robot(self):
        self.robot_ns = '/robot'
        self.robot_model = 'robot'
        self.robot_base_link = 'robot_base_footprint'
        self.move_group_name = 'arm'
        self.ee_link = 'robot_hand_tcp'
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
        self.gripper_open_pos = 0.5

        if self.arm_type.lower() == 'ur5':
            self.move_group_name = 'ur5_arm'
            self.ee_link = 'robot_egh_gripper_grasp_link'
            self.gripper_joint = 'robot_egh_gripper_finger_left_joint'
            self.gripper_open_pos = 0.048

        if rospy.get_param('~robot_ns', '/robot').lower() == '/robot_b':
            self.robot_ns = '/robot_b'
            self.robot_model = 'robot_b'
            self.robot_base_link = 'robot_b_base_footprint'
            self.gripper_cmd_topic = '/robot_b/gripper/pos_traj_controller/command'
            self.arm_cmd_topic = '/robot_b/arm/pos_traj_controller/command'

            if self.arm_type.lower() == 'ur5':
                self.ee_link = 'robot_b_egh_gripper_grasp_link'
                self.gripper_joint = 'robot_b_egh_gripper_finger_left_joint'
            else:
                self.ee_link = 'robot_b_hand_tcp'
                self.gripper_joint = 'robot_b_finger_joint1'
                self.arm_joints = [
                    'robot_b_joint1',
                    'robot_b_joint2',
                    'robot_b_joint3',
                    'robot_b_joint4',
                    'robot_b_joint5',
                    'robot_b_joint6',
                    'robot_b_joint7',
                ]

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

    def create_selected_collision_markers(self):
        markers = MarkerArray()
        marker_stamp = rospy.Time.now()
        marker_color = (0.0, 1.0, 0.0, 0.85)
        mesh_orientation = (0.70710678118, 0.0, 0.0, 0.70710678118)

        def add_sphere_marker(marker_id, name, position, diameter):
            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = marker_stamp
            marker.ns = "selected_collision_markers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter
            marker.color.r = marker_color[0]
            marker.color.g = marker_color[1]
            marker.color.b = marker_color[2]
            marker.color.a = marker_color[3]
            markers.markers.append(marker)

        def add_mesh_marker(marker_id, name, mesh_name, enabled_param, mesh_path_param):
            if not bool(rospy.get_param(enabled_param, True)):
                return

            mesh_resource = rospy.get_param(mesh_path_param, "")
            if mesh_resource:
                if not mesh_resource.startswith("package://") and not mesh_resource.startswith("file://"):
                    mesh_resource = "file://" + mesh_resource
            else:
                mesh_resource = "package://harbot_simulation/models/greenhouse/meshes/" + mesh_name

            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = marker_stamp
            marker.ns = "selected_collision_markers"
            marker.id = marker_id
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = mesh_resource
            marker.pose.orientation.x = mesh_orientation[0]
            marker.pose.orientation.y = mesh_orientation[1]
            marker.pose.orientation.z = mesh_orientation[2]
            marker.pose.orientation.w = mesh_orientation[3]
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = marker_color[0]
            marker.color.g = marker_color[1]
            marker.color.b = marker_color[2]
            marker.color.a = marker_color[3]
            markers.markers.append(marker)

        orange_name = self.target["orange_name"]
        object_position = self.target["object_position"]
        if ".R." in orange_name:
            top_from_bottom = (0.06760, -0.53770, 0.40465)
        else:
            top_from_bottom = (-0.06761, 0.53580, 0.40465)

        if orange_name.endswith(".T"):
            paired_orange_position = [
                object_position[0] - top_from_bottom[0],
                object_position[1] - top_from_bottom[1],
                object_position[2] - top_from_bottom[2],
            ]
        else:
            paired_orange_position = [
                object_position[0] + top_from_bottom[0],
                object_position[1] + top_from_bottom[1],
                object_position[2] + top_from_bottom[2],
            ]

        add_sphere_marker(0, "target_orange", object_position, self.target["diameter"])
        add_sphere_marker(1, "paired_orange", paired_orange_position, self.target["diameter"])
        add_mesh_marker(2, "greenhouse_stems_link", "greenhouse_stems.obj", "~add_greenhouse_stems_collision", "~greenhouse_stems_mesh_path")
        add_mesh_marker(3, "orange_stem", "orange_stem.obj", "~add_orange_stem_collision", "~orange_stem_mesh_path")
        return markers

    def add_mesh_collision_object(
        self,
        scene,
        object_id,
        mesh_name,
        enabled_param,
        mesh_path_param,
        position=(0.0, 0.0, 0.0),
        orientation=(0.70710678118, 0.0, 0.0, 0.70710678118),
        size=(1.0, 1.0, 1.0),
    ):
        if not bool(rospy.get_param(enabled_param, True)):
            return False

        mesh_path = rospy.get_param(mesh_path_param, '')
        if not mesh_path:
            mesh_path = os.path.join(
                rospkg.RosPack().get_path('harbot_simulation'),
                'models',
                'greenhouse',
                'meshes',
                mesh_name,
            )

        if not os.path.isfile(mesh_path):
            # rospy.logwarn('Collision mesh not found for %s: %s', object_id, mesh_path)
            return False

        pose_mesh = PoseStamped()
        pose_mesh.header.frame_id = self.marker_frame
        pose_mesh.pose.position.x = position[0]
        pose_mesh.pose.position.y = position[1]
        pose_mesh.pose.position.z = position[2]
        pose_mesh.pose.orientation.x = orientation[0]
        pose_mesh.pose.orientation.y = orientation[1]
        pose_mesh.pose.orientation.z = orientation[2]
        pose_mesh.pose.orientation.w = orientation[3]
        scene.add_mesh(object_id, pose_mesh, mesh_path, size=size)
        return True

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
        scene.add_box('top_support_box', pose_top_support, size=(0.042, 0.042, 1.32421))

        pose_bottom_support = PoseStamped()
        pose_bottom_support.header.frame_id = self.marker_frame
        pose_bottom_support.pose.position.x = bottom_position[0]
        pose_bottom_support.pose.position.y = bottom_position[1]
        pose_bottom_support.pose.position.z = bottom_position[2] - 0.483475
        pose_bottom_support.pose.orientation.w = 1.0
        scene.add_box('bottom_support_box', pose_bottom_support, size=(0.042, 0.042, 0.919556))

        pose = PoseStamped()
        pose.header.frame_id = self.marker_frame
        pose.pose.position.x = self.target['object_position'][0]
        pose.pose.position.y = self.target['object_position'][1]
        pose.pose.position.z = self.target['object_position'][2]
        pose.pose.orientation.w = 1.0
        scene.add_sphere('detected_object', pose, radius=self.target['diameter'] / 2.0)

        # pose_target_box = PoseStamped()
        # pose_target_box.header.frame_id = self.marker_frame
        # pose_target_box.pose.position.x = object_position[0]
        # pose_target_box.pose.position.y = object_position[1]
        # pose_target_box.pose.position.z = object_position[2] + 0.07
        # pose_target_box.pose.orientation.w = 1.0
        # scene.add_box('target_orange_clearance_box', pose_target_box, size=(0.03, 0.03, 0.07))

        paired_pose = PoseStamped()
        paired_pose.header.frame_id = self.marker_frame
        paired_pose.pose.position.x = paired_orange_position[0]
        paired_pose.pose.position.y = paired_orange_position[1]
        paired_pose.pose.position.z = paired_orange_position[2]
        paired_pose.pose.orientation.w = 1.0
        scene.add_sphere('paired_detected_object', paired_pose, radius=self.target['diameter'] / 2.0)

        self.add_mesh_collision_object(
            scene,
            'greenhouse_stems_link',
            'greenhouse_stems.obj',
            '~add_greenhouse_stems_collision',
            '~greenhouse_stems_mesh_path',
        )
        self.add_mesh_collision_object(
            scene,
            'orange_stem',
            'orange_stem.obj',
            '~add_orange_stem_collision',
            '~orange_stem_mesh_path',
        )

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

    def build_joint_command(self, joint_names, positions, duration):
        cmd = JointTrajectory()
        cmd.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)

        cmd.points = [point]
        return cmd

    def move_arm(self, positions, duration=3.0):
        cmd = self.build_joint_command(
            self.arm_joints,
            positions,
            duration
        )
        self.arm_pub.publish(cmd)

    def scan_position_for_orange(self):
        poses = {
            'back_right': {
                'top': [-0.249, -0.914, -0.461, -2.035, 1.494, 0.906, -0.273],
                'bottom': [-0.429, 0.233, 0.030, -1.904, 1.325, 1.211, -1.360],
            },
            'back_left': {
                'top': [-0.125, 0.653, 0.982, -0.622, -2.408, 1.137, 1.831],
                'bottom': [0.163, -1.037, 0.807, -2.724, -0.755, 1.074, 2.606],
            },
            'front_right': {
                'top': [-0.241, -1.294, -0.601, -1.975, 1.576, 0.852, 0.225],
                'bottom': [-0.510, -0.155, -0.050, -2.357, 1.190, 1.105, -1.470],
            },
            'front_left': {
                'top': [-0.734, -0.632, 0.671, -1.777, -1.178, 1.575, 2.058],
                'bottom': [-2.086, -1.062, -1.309, -2.582, -2.480, 4.206, 2.007],
            },
        }

        try:
            orange_id, orange_index, side, height = self.orange_model.split('.')
            row = int(orange_id.replace('orange', ''))
            index = int(orange_index)
            side = side.upper()
            height = height.upper()
            if index < 1 or index > 8:
                raise ValueError
        except ValueError as exc:
            raise RuntimeError('Invalid orange model name: {}'.format(self.orange_model)) from exc

        if side == 'L':
            if row in (1, 4) or (row == 3 and index <= 4):
                pose_group = 'back_right'
            elif row == 2 or (row == 3 and index >= 5):
                pose_group = 'front_left'
            else:
                raise RuntimeError('No scan pose for {}'.format(self.orange_model))
        elif side == 'R':
            if row == 3 or (row == 2 and index <= 4):
                pose_group = 'back_left'
            elif row in (1, 4) or (row == 2 and index >= 5):
                pose_group = 'front_right'
            else:
                raise RuntimeError('No scan pose for {}'.format(self.orange_model))
        else:
            raise RuntimeError('No scan pose for {}'.format(self.orange_model))

        if height == 'B':
            return poses[pose_group]['bottom']
        if height == 'T':
            return poses[pose_group]['top']

        raise RuntimeError('No scan pose for {}'.format(self.orange_model))

    def move_arm_to_scan_position(self):
        if self.arm_type.lower() != 'franka':
            rospy.logwarn('Scan pose is only configured for Franka; skipping')
            return

        self.move_arm(self.scan_position_for_orange())
        rospy.loginfo('Moved arm to scan pose for %s', self.orange_model)

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
        
        attached = False

        for attempt in range(5):
            if self.set_link_attachment(
                'attach',
                self.robot_model,
                self.robot_base_link,
                self.orange_model,
                'body',
            ):
                rospy.sleep(0.1)
                attached = True
                break

            rospy.sleep(0.1)

        if not attached:
            moveit_commander.roscpp_shutdown()
            return False
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

        # rospy.sleep(1.0)
        self.add_collision_objects(scene)
        rospy.sleep(1.0)

        group.set_pose_reference_frame(self.marker_frame)
        # group.set_start_state_to_current_state()
        # group.allow_replanning(True)    
        group.set_position_target(self.target['object_position'], resolved_ee_link)

        plan = None
        for attempt in range(1, no_solution_retries + 2):
            rospy.loginfo(
                'Planning to target (attempt %d/%d)',
                attempt,
                no_solution_retries + 1,
            )

            planning_start = time.monotonic()

            RED = "\033[91m"
            RESET = "\033[0m"

            # rospy.loginfo(
            #     "%s[MOVEIT TIMER START] attempt %d/%d at %.6f%s",
            #     RED,
            #     attempt,
            #     no_solution_retries + 1,
            #     planning_start,
            #     RESET,
            # )

            planned, candidate_plan = self.extract_plan_result(group.plan())

            planning_elapsed = time.monotonic() - planning_start
            self.moveit_planning_time_sec += planning_elapsed

            # rospy.loginfo(
            #     "%s[MOVEIT TIMER STOP] attempt %d/%d elapsed %.3f sec | total %.3f sec%s",
            #     RED,
            #     attempt,
            #     no_solution_retries + 1,
            #     planning_elapsed,
            #     self.moveit_planning_time_sec,
            #     RESET,
            # )
            if planned and self.plan_has_trajectory(candidate_plan):
                plan = candidate_plan
                planning_done_time = time.monotonic()
                # rospy.loginfo(
                #     "\033[91m[POST MOVEIT TIMER START] %s at %.6f\033[0m",
                #     self.orange_model,
                #     planning_done_time,
                # )
                self.moveit_planning_done_time_pub.publish(Float64(data=planning_done_time))
                break

            if attempt <= no_solution_retries:
                # rospy.logwarn('MoveIt found no plan; opening gripper before replanning')
                self.open_gripper(gripper_pub)
                rospy.sleep(0.1)

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


        self.configure_robot()
        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        rospy.sleep(0.5)
        self.move_arm_to_scan_position()
        rospy.sleep(5.0)

        detection_processes = []
        try:
            detection_processes = self.launch_detection_processes()
            self.load_orange_detection()
        except Exception as exc:
            rospy.logerr('Failed to get detected orange pose: %s', str(exc))
            return
        finally:
            self.shutdown_detection_processes(detection_processes)

        object_marker = self.create_object_marker()
        target_marker = self.create_target_marker()
        selected_collision_markers = self.create_selected_collision_markers()

        moved = False
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            marker_stamp = rospy.Time.now()
            object_marker.header.stamp = marker_stamp
            target_marker.header.stamp = marker_stamp
            for selected_marker in selected_collision_markers.markers:
                selected_marker.header.stamp = marker_stamp

            self.object_marker_pub.publish(object_marker)
            self.target_marker_pub.publish(target_marker)
            self.selected_collision_marker_pub.publish(selected_collision_markers)

            if not moved:
                moved = True
                success = self.move_arm_to_target()
                self.moveit_planning_time_pub.publish(Float64(data=self.moveit_planning_time_sec))
                self.orange_action_pub.publish(Bool(data=success))
                return

            rate.sleep()


def main():
    rospy.init_node('moveit_target_reach', anonymous=True)
    MoveItTargetReach().run()


if __name__ == '__main__':
    main()
