#!/usr/bin/env python3

import json
import math
import time
import rospy
import threading
import tkinter as tk
from tkinter import ttk
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Bool, String
from tf.transformations import euler_from_quaternion


DEFAULT_WAYPOINTS = {
    'robot_a': {
        'a1': (-7.5, -12.0, 1.5708),
        'a2': (-7.5, -8.559429, 1.5708),
        'a3': (-7.5, -5.118858, 1.5708),
        'a4': (-7.5, -1.678287, 1.5708),
        'a5': (-7.5, 1.762284, 1.5708),
        'a6': (-7.5, 5.202855, 1.5708),
        'a7': (-7.5, 8.643426, 1.5708),
        'a8': (-7.5, 12.083997, 1.5708),
        'a16': (-4.0, 12.083997, -1.5708),
        'a15': (-4.0, 8.643426, -1.5708),
        'a14': (-4.0, 5.202855, -1.5708),
        'a13': (-4.0, 1.762284, -1.5708),
        'a12': (-4.0, -1.678287, -1.5708),
        'a11': (-4.0, -5.118858, -1.5708),
        'a10': (-4.0, -8.559429, -1.5708),
        'a9': (-4.0, -12.0, -1.5708),
        'a17': (0.0, -12.0, 1.5708),
        'a18': (0.0, -8.559429, 1.5708),
        'a19': (0.0, -5.118858, 1.5708),
        'a20': (0.0, -1.678287, 1.5708),
    },
    'robot_b': {
        'a21': (0.0, 1.762284, -1.5708),
        'a22': (0.0, 5.202855, -1.5708),
        'a23': (0.0, 8.643426, -1.5708),
        'a24': (0.0, 12.083997, -1.5708),
        'a32': (4.0, 12.083997, 1.5708),
        'a31': (4.0, 8.643426, 1.5708),
        'a30': (4.0, 5.202855, 1.5708),
        'a29': (4.0, 1.762284, 1.5708),
        'a28': (4.0, -1.678287, 1.5708),
        'a27': (4.0, -5.118858, 1.5708),
        'a26': (4.0, -8.559429, 1.5708),
        'a25': (4.0, -12.0, 1.5708),
        'a33': (7.5, -12.0, -1.5708),
        'a34': (7.5, -8.559429, -1.5708),
        'a35': (7.5, -5.118858, -1.5708),
        'a36': (7.5, -1.678287, -1.5708),
        'a37': (7.5, 1.762284, -1.5708),
        'a38': (7.5, 5.202855, -1.5708),
        'a39': (7.5, 8.643426, -1.5708),
        'a40': (7.5, 12.083997, -1.5708),
    },
}

WAYPOINT_STYLES = {
    'robot_a': {
        'fill': '#8ec5ff',
        'outline': '#1455a3',
        'text': '#0f3f78',
        'visited_fill': '#1f6fd1',
        'visited_outline': '#0b3d91',
        'visited_highlight': '#4f9cf5',
        'visited_text': '#ffffff',
        'legend': 'Robot A Waypoints',
    },
    'robot_b': {
        'fill': '#ff9b9b',
        'outline': '#9d2424',
        'text': '#751919',
        'visited_fill': '#c62828',
        'visited_outline': '#7f1515',
        'visited_highlight': '#ef5350',
        'visited_text': '#ffffff',
        'legend': 'Robot B Waypoints',
    },
}

DEFAULT_UNLOADING_STATIONS = {
    'ul1': {
        'rect': (-8.175, -14.830, -6.825, -13.970),
        'style': 'robot_a',
    },
    'ul2': {
        'rect': (6.825, 13.970, 8.175, 14.830),
        'style': 'robot_b',
    },
}


def waypoint_sort_key(waypoint_name):
    lowered = waypoint_name.lower()
    if lowered.startswith('a') and lowered[1:].isdigit():
        return int(lowered[1:])
    return lowered


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


class PathGuiNode(object):
    def __init__(self):
        namespace = rospy.get_namespace().strip('/')
        self.robot_namespace = namespace or 'robot'
        self.is_robot_a = self.robot_namespace in ('robot', 'robot_a')
        self.other_robot_namespace = 'robot_b' if self.is_robot_a else 'robot'

        self.odom_topic = rospy.get_param('~odom_topic', 'robotnik_base_control/odom_gt')
        self.other_odom_topic = rospy.get_param(
            '~other_odom_topic',
            '/{}/robotnik_base_control/odom_gt'.format(self.other_robot_namespace)
        )
        self.global_path_topic = rospy.get_param('~global_path_topic', 'path_nav/path')
        self.desired_velocity_topic = rospy.get_param('~desired_velocity_topic', 'path_nav/desired_speed')
        self.state_flag_topic = rospy.get_param('~state_flag_topic', 'path_nav/status')
        self.proceed_flag_topic = rospy.get_param('~proceed_flag_topic', 'path_nav/proceed_flag')
        self.reverse_mode_topic = rospy.get_param('~reverse_mode_topic', 'path_nav/reverse_mode')
        self.completed_waypoints_topic = rospy.get_param(
            '~completed_waypoints_topic',
            'path_nav/completed_waypoints'
        )
        self.other_completed_waypoints_topic = rospy.get_param(
            '~other_completed_waypoints_topic',
            '/{}/path_nav/completed_waypoints'.format(self.other_robot_namespace)
        )
        self.wheel_base = float(rospy.get_param('~wheel_base', 0.45))
        self.vehicle_width = float(rospy.get_param('~vehicle_width', 0.60))
        self.vehicle_length = float(rospy.get_param('~vehicle_length', 0.70))
        self.path_name = rospy.get_param('~path_name', 'example_straight.json')
        self.path_clearance_radius = float(rospy.get_param('~path_clearance_radius', 0.72))
        self.default_show_waypoints = parse_bool(rospy.get_param('~show_waypoints', True))
        self.default_show_waypoint_labels = parse_bool(rospy.get_param('~show_waypoint_labels', True))
        self.default_show_unloading_stations = parse_bool(rospy.get_param('~show_unloading_stations', True))
        self.waypoint_marker_radius = float(rospy.get_param('~waypoint_marker_radius', 0.36))
        self.unloading_stations = DEFAULT_UNLOADING_STATIONS
        self.waypoints = self.load_waypoints()
        self.waypoint_count = sum(len(robot_points) for robot_points in self.waypoints.values())
        self.completed_waypoints = {'robot_a': set(), 'robot_b': set()}

        self.vehicle_position = [0.0, 0.0, 0.0]
        self.other_vehicle_position = []
        self.vehicle_velocity = 0.0
        self.path_points = []
        self.desired_speed = 0.0
        self.state = 'IDLE'
        self.proceed_flag = False
        self.scale = 30.0
        self.reverse_mode = False
        self.self_vehicle_color = 'blue' if self.is_robot_a else 'red'
        self.other_vehicle_color = 'red' if self.is_robot_a else 'blue'

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.other_odom_topic, Odometry, self.other_odom_callback, queue_size=10)
        rospy.Subscriber(self.global_path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.desired_velocity_topic, Float64, self.velocity_callback, queue_size=1)
        rospy.Subscriber(self.state_flag_topic, String, self.state_callback, queue_size=1)
        rospy.Subscriber(self.proceed_flag_topic, Bool, self.proceed_callback, queue_size=1)
        rospy.Subscriber(self.reverse_mode_topic, Bool, self.reverse_mode_callback, queue_size=1)
        rospy.Subscriber(
            self.completed_waypoints_topic,
            String,
            self.completed_waypoints_callback,
            queue_size=1
        )
        rospy.Subscriber(
            self.other_completed_waypoints_topic,
            String,
            self.other_completed_waypoints_callback,
            queue_size=1
        )

        self.proceed_pub = rospy.Publisher(self.proceed_flag_topic, Bool, queue_size=1, latch=True)

        threading.Thread(target=self.start_gui, daemon=True).start()

    def reverse_mode_callback(self, msg):
        self.reverse_mode = msg.data

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ]
        linear = msg.twist.twist.linear
        self.vehicle_velocity = math.sqrt(linear.x ** 2 + linear.y ** 2)

    def other_odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.other_vehicle_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ]

    def path_callback(self, msg):
        self.path_points = []
        for pose in msg.poses:
            q = pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.path_points.append([
                pose.pose.position.x,
                pose.pose.position.y,
                yaw
            ])

    def velocity_callback(self, msg):
        self.desired_speed = msg.data

    def state_callback(self, msg):
        self.state = msg.data

    def proceed_callback(self, msg):
        self.proceed_flag = msg.data
        if self.proceed_flag:
            self.state = 'RUNNING'

    def start_planner(self):
        self.proceed_pub.publish(Bool(data=True))
        self.proceed_flag = True

    def stop_planner(self):
        self.proceed_pub.publish(Bool(data=False))
        self.proceed_flag = False

    def load_waypoints(self):
        raw_waypoints = rospy.get_param('~waypoints', DEFAULT_WAYPOINTS)
        if not isinstance(raw_waypoints, dict):
            rospy.logwarn('Waypoint parameter is not a dictionary; using default waypoints')
            raw_waypoints = DEFAULT_WAYPOINTS

        normalized = {}
        for robot_key, robot_waypoints in raw_waypoints.items():
            if not isinstance(robot_waypoints, dict):
                rospy.logwarn('Ignoring waypoint group %s because it is not a dictionary', robot_key)
                continue

            normalized[robot_key] = {}
            for waypoint_name, pose in robot_waypoints.items():
                try:
                    x_value, y_value, yaw = self.parse_waypoint_pose(pose)
                except (TypeError, ValueError, IndexError) as exc:
                    rospy.logwarn('Ignoring waypoint %s/%s: %s', robot_key, waypoint_name, exc)
                    continue
                normalized[robot_key][str(waypoint_name).lower()] = (x_value, y_value, yaw)

        return normalized

    def parse_waypoint_pose(self, pose):
        if isinstance(pose, dict):
            return (
                float(pose['x']),
                float(pose['y']),
                float(pose.get('yaw', 0.0)),
            )

        if len(pose) < 2:
            raise ValueError('expected at least x and y')

        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        return float(pose[0]), float(pose[1]), yaw

    def completed_waypoints_callback(self, msg):
        robot_key = 'robot_a' if self.is_robot_a else 'robot_b'
        self.update_completed_waypoints(robot_key, msg)

    def other_completed_waypoints_callback(self, msg):
        robot_key = 'robot_b' if self.is_robot_a else 'robot_a'
        self.update_completed_waypoints(robot_key, msg)

    def update_completed_waypoints(self, robot_key, msg):
        self.completed_waypoints[robot_key] = self.parse_completed_waypoints_message(msg.data)

    def parse_completed_waypoints_message(self, message):
        try:
            payload = json.loads(message)
        except (TypeError, ValueError):
            payload = message

        if isinstance(payload, dict):
            waypoint_names = payload.get('completed_waypoints', [])
        elif isinstance(payload, list):
            waypoint_names = payload
        else:
            waypoint_names = str(payload).split(',')

        return {
            str(waypoint_name).strip().lower()
            for waypoint_name in waypoint_names
            if str(waypoint_name).strip()
        }

    def start_gui(self):
        self.root = tk.Tk()
        namespace = rospy.get_namespace().strip('/') or 'global'
        self.root.title(f'ORBit Visualization - {namespace}')

        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        self.root.geometry('1920x1080+0+0')

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.Frame(main_frame, padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X)

        status_frame = ttk.LabelFrame(control_frame, text='Status', padding=10)
        status_frame.pack(side=tk.LEFT, fill=tk.Y)

        self.velocity_var = tk.StringVar(value='Velocity: 0.0 m/s')
        ttk.Label(status_frame, textvariable=self.velocity_var).pack(anchor=tk.W)

        self.desired_velocity_var = tk.StringVar(value='Desired: 0.0 m/s')
        ttk.Label(status_frame, textvariable=self.desired_velocity_var).pack(anchor=tk.W)

        self.state_var = tk.StringVar(value='State: IDLE')
        self.state_label = ttk.Label(status_frame, textvariable=self.state_var)
        self.state_label.pack(anchor=tk.W)

        # self.proceed_var = tk.StringVar(value='Proceed: False')
        # ttk.Label(status_frame, textvariable=self.proceed_var).pack(anchor=tk.W)

        self.position_var = tk.StringVar(value='Position: (0.0, 0.0)')
        ttk.Label(status_frame, textvariable=self.position_var).pack(anchor=tk.W)

        self.other_position_var = tk.StringVar(value='Other: n/a')
        ttk.Label(status_frame, textvariable=self.other_position_var).pack(anchor=tk.W)

        # self.path_info_var = tk.StringVar(value='Path Points: 0')
        # ttk.Label(status_frame, textvariable=self.path_info_var).pack(anchor=tk.W)

        button_frame = ttk.Frame(control_frame, padding=10)
        button_frame.pack(side=tk.RIGHT)

        button_row_top = ttk.Frame(button_frame)
        button_row_top.pack(anchor=tk.E, pady=(0, 4))

        button_row_bottom = ttk.Frame(button_frame)
        button_row_bottom.pack(anchor=tk.E)

        ttk.Button(button_row_top, text='Center View', command=self.center_view).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_row_top, text='Zoom In', command=self.zoom_in).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_row_top, text='Zoom Out', command=self.zoom_out).pack(side=tk.LEFT, padx=5)

        ttk.Button(button_row_bottom, text='Start', command=self.start_planner).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_row_bottom, text='Stop', command=self.stop_planner).pack(side=tk.LEFT, padx=5)

        self.scale_var = tk.StringVar(value=f'Scale: {self.scale:.1f} px/m')
        ttk.Label(button_row_bottom, textvariable=self.scale_var).pack(side=tk.LEFT, padx=10)

        options_frame = ttk.LabelFrame(control_frame, text='View Options', padding=10)
        options_frame.pack(side=tk.LEFT, fill=tk.Y, padx=20)

        self.show_path_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text='Show Path', variable=self.show_path_var).pack(anchor=tk.W)

        self.show_grid_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text='Show Grid', variable=self.show_grid_var).pack(anchor=tk.W)

        # self.setup_extra_view_options(options_frame)

        self.follow_vehicle_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            options_frame,
            text='Follow Vehicle',
            variable=self.follow_vehicle_var,
            command=self.toggle_follow_vehicle
        ).pack(anchor=tk.W)

        legend_frame = ttk.LabelFrame(control_frame, text='Legend', padding=10)
        legend_frame.pack(side=tk.LEFT, fill=tk.Y, padx=20)

        legend_top_row = ttk.Frame(legend_frame)
        legend_top_row.pack(anchor=tk.W)

        robot_a_canvas = tk.Canvas(legend_top_row, width=15, height=15, highlightthickness=0)
        robot_a_canvas.create_rectangle(0, 0, 15, 15, fill='blue')
        robot_a_canvas.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_top_row, text='Robot A').pack(side=tk.LEFT)

        robot_b_canvas = tk.Canvas(legend_top_row, width=15, height=15, highlightthickness=0)
        robot_b_canvas.create_rectangle(0, 0, 15, 15, fill='red')
        robot_b_canvas.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_top_row, text='Robot B').pack(side=tk.LEFT)

        path_canvas = tk.Canvas(legend_top_row, width=15, height=15, highlightthickness=0)
        path_canvas.create_rectangle(0, 0, 15, 15, fill='green')
        path_canvas.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_top_row, text='Path').pack(side=tk.LEFT)

        area_canvas = tk.Canvas(legend_top_row, width=16, height=16, highlightthickness=0)
        area_canvas.create_oval(2, 2, 14, 14, outline='#7fbf7f', dash=(2, 2))
        area_canvas.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_top_row, text='Path Area').pack(side=tk.LEFT)

        legend_bottom_row = ttk.Frame(legend_frame)
        legend_bottom_row.pack(anchor=tk.W, pady=(6, 0))

        forward_line = tk.Canvas(legend_bottom_row, width=22, height=15, highlightthickness=0)
        forward_line.create_line(2, 8, 20, 8, fill='deepskyblue', width=3, arrow=tk.LAST)
        forward_line.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_bottom_row, text='Forward').pack(side=tk.LEFT, padx=(0, 12))

        reverse_line = tk.Canvas(legend_bottom_row, width=22, height=15, highlightthickness=0)
        reverse_line.create_line(20, 8, 2, 8, fill='orange', width=3, arrow=tk.LAST)
        reverse_line.pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_bottom_row, text='Reverse').pack(side=tk.LEFT)

        self.setup_extra_legend(legend_frame)

        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, background='white')
        v_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.configure(xscrollcommand=h_scrollbar.set, yscrollcommand=v_scrollbar.set)
        self.canvas.configure(scrollregion=(-5000, -5000, 5000, 5000))

        self.canvas.bind('<ButtonPress-1>', self.scroll_start)
        self.canvas.bind('<B1-Motion>', self.scroll_move)
        self.canvas.bind('<MouseWheel>', self.on_mousewheel)
        self.canvas.bind('<Button-4>', self.on_mousewheel)
        self.canvas.bind('<Button-5>', self.on_mousewheel)

        self.root.bind('<Escape>', lambda _e: self.root.quit())

        self.initial_centering_time = time.time()
        self.initial_centering_done = False

        self.root.update_idletasks()
        self.center_on_world(0.0, 0.0)

        self.update_gui()
        self.root.mainloop()

    def scroll_start(self, event):
        self.canvas.scan_mark(event.x, event.y)

    def scroll_move(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        self.follow_vehicle_var.set(False)
        self.initial_centering_done = True

    def center_view(self):
        if not self.vehicle_position:
            return

        canvas_x, canvas_y = self.world_to_canvas(self.vehicle_position[0], self.vehicle_position[1])
        canvas_width = self.canvas.winfo_width() or 800
        canvas_height = self.canvas.winfo_height() or 600
        scroll_size = 10000

        scroll_left = canvas_x - scroll_size / 2
        scroll_top = canvas_y - scroll_size / 2
        scroll_right = canvas_x + scroll_size / 2
        scroll_bottom = canvas_y + scroll_size / 2
        self.canvas.configure(scrollregion=(scroll_left, scroll_top, scroll_right, scroll_bottom))

        x_center = (canvas_x - scroll_left) / (scroll_right - scroll_left)
        y_center = (canvas_y - scroll_top) / (scroll_bottom - scroll_top)
        x_view = max(0.0, min(1.0, x_center - (canvas_width / (scroll_right - scroll_left)) / 2))
        y_view = max(0.0, min(1.0, y_center - (canvas_height / (scroll_bottom - scroll_top)) / 2))

        self.canvas.xview_moveto(x_view)
        self.canvas.yview_moveto(y_view)

    def center_on_world(self, x_value=0.0, y_value=0.0):
        canvas_x, canvas_y = self.world_to_canvas(x_value, y_value)
        canvas_width = self.canvas.winfo_width() or 800
        canvas_height = self.canvas.winfo_height() or 600
        scroll_left, scroll_top, scroll_right, scroll_bottom = -5000, -5000, 5000, 5000

        self.canvas.configure(scrollregion=(scroll_left, scroll_top, scroll_right, scroll_bottom))

        x_center = (canvas_x - scroll_left) / (scroll_right - scroll_left)
        y_center = (canvas_y - scroll_top) / (scroll_bottom - scroll_top)
        x_view = max(0.0, min(1.0, x_center - (canvas_width / (scroll_right - scroll_left)) / 2))
        y_view = max(0.0, min(1.0, y_center - (canvas_height / (scroll_bottom - scroll_top)) / 2))

        self.canvas.xview_moveto(x_view)
        self.canvas.yview_moveto(y_view)

    def toggle_follow_vehicle(self):
        if self.follow_vehicle_var.get():
            self.initial_centering_done = False
            self.initial_centering_time = time.time()
            self.center_view()

    def zoom_in(self):
        self.scale *= 1.2
        self.scale_var.set(f'Scale: {self.scale:.1f} px/m')
        self.update_gui_once()
        self.center_view()

    def zoom_out(self):
        self.scale /= 1.2
        self.scale_var.set(f'Scale: {self.scale:.1f} px/m')
        self.update_gui_once()
        self.center_view()

    def on_mousewheel(self, event):
        zoom_in = False
        if hasattr(event, 'num'):
            zoom_in = event.num == 4
        elif hasattr(event, 'delta'):
            zoom_in = event.delta > 0

        if zoom_in:
            self.zoom_in()
        else:
            self.zoom_out()

    def world_to_canvas(self, x_value, y_value):
        canvas_x = x_value * self.scale
        canvas_y = -y_value * self.scale
        return canvas_x, canvas_y

    def draw_vehicle_shape(self, position, fill_color, tags, draw_direction=False, direction_color='black', direction_arrow=tk.LAST, direction_width=2):
        if not position:
            return

        x_value, y_value, yaw = position
        length = self.vehicle_length
        width = self.vehicle_width

        corners = [
            [-width / 2.0, -length / 2.0],
            [width / 2.0, -length / 2.0],
            [width / 2.0, length / 2.0],
            [-width / 2.0, length / 2.0]
        ]

        rotated_corners = []
        for corner_x, corner_y in corners:
            rot_x = corner_x * math.cos(yaw) - corner_y * math.sin(yaw)
            rot_y = corner_x * math.sin(yaw) + corner_y * math.cos(yaw)
            canvas_x, canvas_y = self.world_to_canvas(x_value + rot_x, y_value + rot_y)
            rotated_corners.extend([canvas_x, canvas_y])

        self.canvas.create_polygon(rotated_corners, fill=fill_color, outline='black', width=1, tags=tags)

        if draw_direction:
            front_x = x_value + min(length * 0.45, 0.35) * math.cos(yaw)
            front_y = y_value + min(length * 0.45, 0.35) * math.sin(yaw)
            canvas_x, canvas_y = self.world_to_canvas(x_value, y_value)
            canvas_front_x, canvas_front_y = self.world_to_canvas(front_x, front_y)
            self.canvas.create_line(
                canvas_x,
                canvas_y,
                canvas_front_x,
                canvas_front_y,
                fill=direction_color,
                width=direction_width,
                arrow=direction_arrow,
                tags=tags
            )

    def draw_vehicle(self):
        if not self.vehicle_position:
            return

        direction_color = 'orange' if self.reverse_mode else 'deepskyblue'
        direction_arrow = tk.FIRST if self.reverse_mode else tk.LAST
        self.draw_vehicle_shape(
            self.vehicle_position,
            self.self_vehicle_color,
            'vehicle',
            draw_direction=True,
            direction_color=direction_color,
            direction_arrow=direction_arrow,
            direction_width=3
        )

    def draw_other_vehicle(self):
        if not self.other_vehicle_position:
            return

        self.draw_vehicle_shape(
            self.other_vehicle_position,
            self.other_vehicle_color,
            'other_vehicle',
            draw_direction=True,
            direction_color='black',
            direction_arrow=tk.LAST,
            direction_width=2
        )

    def draw_path(self):
        if not self.path_points or not self.show_path_var.get():
            return

        point_radius = max(1.5, min(3.0, self.scale * 0.04))
        line_width = max(1, min(2, int(self.scale * 0.03)))
        area_radius = max(2.0, self.path_clearance_radius * self.scale)

        points = []
        for point in self.path_points:
            canvas_x, canvas_y = self.world_to_canvas(point[0], point[1])
            points.extend([canvas_x, canvas_y])
            self.canvas.create_oval(
                canvas_x - area_radius,
                canvas_y - area_radius,
                canvas_x + area_radius,
                canvas_y + area_radius,
                outline='#7fbf7f',
                dash=(2, 2),
                width=1,
                tags='path_area'
            )

        if len(points) >= 4:
            self.canvas.create_line(points, fill='green', width=line_width, tags='path')

        for point in self.path_points:
            canvas_x, canvas_y = self.world_to_canvas(point[0], point[1])
            self.canvas.create_oval(
                canvas_x - point_radius,
                canvas_y - point_radius,
                canvas_x + point_radius,
                canvas_y + point_radius,
                fill='green',
                outline='green',
                tags='path'
            )

    def setup_extra_view_options(self, options_frame):
        self.show_waypoints_var = tk.BooleanVar(value=self.default_show_waypoints)
        ttk.Checkbutton(
            options_frame,
            text='Waypoint Markers',
            variable=self.show_waypoints_var
        ).pack(anchor=tk.W)

        self.show_waypoint_labels_var = tk.BooleanVar(value=self.default_show_waypoint_labels)
        ttk.Checkbutton(
            options_frame,
            text='Waypoint Names',
            variable=self.show_waypoint_labels_var
        ).pack(anchor=tk.W)

        self.show_unloading_stations_var = tk.BooleanVar(value=self.default_show_unloading_stations)
        ttk.Checkbutton(
            options_frame,
            text='Unloading Stations',
            variable=self.show_unloading_stations_var
        ).pack(anchor=tk.W)

    def setup_extra_legend(self, legend_frame):
        waypoint_row = ttk.Frame(legend_frame)
        waypoint_row.pack(anchor=tk.W, pady=(6, 0))

        for robot_key in ('robot_a', 'robot_b'):
            style = WAYPOINT_STYLES[robot_key]
            marker_canvas = tk.Canvas(waypoint_row, width=22, height=18, highlightthickness=0)
            marker_canvas.create_oval(
                3,
                2,
                17,
                16,
                fill=style['fill'],
                outline=style['outline'],
                width=2,
            )
            marker_canvas.pack(side=tk.LEFT, padx=5)
            ttk.Label(waypoint_row, text=style['legend']).pack(side=tk.LEFT, padx=(0, 12))

        station_row = ttk.Frame(legend_frame)
        station_row.pack(anchor=tk.W, pady=(6, 0))

        for station_name in ('ul1', 'ul2'):
            style = self.unloading_station_style(self.unloading_stations[station_name])
            station_canvas = tk.Canvas(station_row, width=24, height=16, highlightthickness=0)
            station_canvas.create_rectangle(
                2,
                3,
                22,
                13,
                fill=style['fill'],
                outline=style['outline'],
                width=2,
            )
            station_canvas.pack(side=tk.LEFT, padx=5)
            ttk.Label(station_row, text=station_name.upper()).pack(side=tk.LEFT, padx=(0, 12))

    def waypoints_visible(self):
        var = getattr(self, 'show_waypoints_var', None)
        return self.default_show_waypoints if var is None else var.get()

    def waypoint_labels_visible(self):
        var = getattr(self, 'show_waypoint_labels_var', None)
        return self.default_show_waypoint_labels if var is None else var.get()

    def unloading_stations_visible(self):
        var = getattr(self, 'show_unloading_stations_var', None)
        return self.default_show_unloading_stations if var is None else var.get()

    def draw_extra_map_layers(self):
        self.draw_unloading_stations()
        self.draw_waypoints()

    def unloading_station_style(self, station):
        return WAYPOINT_STYLES.get(station.get('style', 'robot_a'), WAYPOINT_STYLES['robot_a'])

    def draw_unloading_stations(self):
        if not self.unloading_stations_visible():
            return

        for station_name, station in self.unloading_stations.items():
            style = self.unloading_station_style(station)
            x1, y1, x2, y2 = station['rect']
            canvas_x1, canvas_y1 = self.world_to_canvas(x1, y1)
            canvas_x2, canvas_y2 = self.world_to_canvas(x2, y2)
            left = min(canvas_x1, canvas_x2)
            right = max(canvas_x1, canvas_x2)
            top = min(canvas_y1, canvas_y2)
            bottom = max(canvas_y1, canvas_y2)

            self.canvas.create_rectangle(
                left,
                top,
                right,
                bottom,
                fill=style['fill'],
                outline=style['outline'],
                width=3,
                tags='unloading_station',
            )
            self.canvas.create_text(
                (left + right) / 2.0,
                (top + bottom) / 2.0,
                text=station_name.upper(),
                fill=style['text'],
                font=('TkDefaultFont', 9, 'bold'),
                tags='unloading_station_label',
            )

    def draw_waypoints(self):
        if not self.waypoints_visible():
            return

        marker_radius = max(24.0, min(120.0, self.waypoint_marker_radius * self.scale))
        outline_width = max(2, min(5, int(marker_radius * 0.08)))
        label_font_size = int(max(10, min(22, marker_radius * 0.30)))

        for robot_key in ('robot_a', 'robot_b'):
            style = WAYPOINT_STYLES.get(robot_key, WAYPOINT_STYLES['robot_a'])
            robot_waypoints = self.waypoints.get(robot_key, {})
            for waypoint_name in sorted(robot_waypoints, key=waypoint_sort_key):
                x_value, y_value, _yaw = robot_waypoints[waypoint_name]
                canvas_x, canvas_y = self.world_to_canvas(x_value, y_value)
                label = waypoint_name.upper()
                is_completed = waypoint_name in self.completed_waypoints.get(robot_key, set())
                marker_fill = style['visited_fill'] if is_completed else style['fill']
                marker_outline = style['visited_outline'] if is_completed else style['outline']
                label_fill = style['visited_text'] if is_completed else style['text']

                self.canvas.create_oval(
                    canvas_x - marker_radius,
                    canvas_y - marker_radius,
                    canvas_x + marker_radius,
                    canvas_y + marker_radius,
                    fill=marker_fill,
                    outline=marker_outline,
                    width=outline_width,
                    tags='waypoint',
                )

                if is_completed:
                    highlight_radius = marker_radius * 0.58
                    self.canvas.create_oval(
                        canvas_x - highlight_radius,
                        canvas_y - highlight_radius,
                        canvas_x + highlight_radius,
                        canvas_y + highlight_radius,
                        fill=style['visited_highlight'],
                        outline=style['visited_highlight'],
                        width=1,
                        tags='waypoint',
                    )

                if self.waypoint_labels_visible():
                    self.canvas.create_text(
                        canvas_x,
                        canvas_y,
                        text=label,
                        anchor=tk.CENTER,
                        fill=label_fill,
                        font=('TkDefaultFont', label_font_size, 'bold'),
                        tags='waypoint_label',
                    )

    def draw_grid(self):
        if not self.show_grid_var.get():
            return

        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 600
        scroll_region = self.canvas.cget('scrollregion').split()
        if len(scroll_region) == 4:
            x1, y1, x2, y2 = map(float, scroll_region)
        else:
            x1, y1, x2, y2 = -5000, -5000, 5000, 5000

        view_x1 = self.canvas.canvasx(0)
        view_y1 = self.canvas.canvasy(0)
        view_x2 = self.canvas.canvasx(width)
        view_y2 = self.canvas.canvasy(height)

        grid_spacing = 10 * self.scale
        while grid_spacing < 20:
            grid_spacing *= 2

        self.canvas.create_line(x1, 0, x2, 0, fill='red', width=2, tags='grid')
        self.canvas.create_line(0, y1, 0, y2, fill='blue', width=2, tags='grid')

        start_x = math.floor(view_x1 / grid_spacing) * grid_spacing
        start_y = math.floor(view_y1 / grid_spacing) * grid_spacing

        x_value = start_x
        while x_value <= view_x2:
            if abs(x_value) > 1:
                self.canvas.create_line(x_value, view_y1, x_value, view_y2, fill='#DDDDDD', tags='grid')
            x_value += grid_spacing

        y_value = start_y
        while y_value <= view_y2:
            if abs(y_value) > 1:
                self.canvas.create_line(view_x1, y_value, view_x2, y_value, fill='#DDDDDD', tags='grid')
            y_value += grid_spacing

    def update_status_display(self):
        self.velocity_var.set(f'Velocity: {self.vehicle_velocity:.2f} m/s')
        self.desired_velocity_var.set(f'Desired: {self.desired_speed:.2f} m/s')
        self.state_var.set(f'State: {self.state}')
        # self.proceed_var.set(f'Proceed: {self.proceed_flag}')
        # self.path_info_var.set(f'Path Points: {len(self.path_points)} | Waypoints: {self.waypoint_count}')

        if self.vehicle_position:
            self.position_var.set(f'Position: ({self.vehicle_position[0]:.2f}, {self.vehicle_position[1]:.2f})')
        if self.other_vehicle_position:
            self.other_position_var.set(
                f'Other: ({self.other_vehicle_position[0]:.2f}, {self.other_vehicle_position[1]:.2f})'
            )
        else:
            self.other_position_var.set('Other: n/a')

        if self.state == 'RUNNING':
            self.state_label.configure(foreground='green')
        elif self.state == 'STOPPED_AT_POINT':
            self.state_label.configure(foreground='orange')
        elif self.state in ('WAITING_FOR_PATH_CLEAR', 'WAITING_FOR_CLEAR_CONFIRMATION'):
            self.state_label.configure(foreground='darkorange')
        elif self.state == 'STOPPED':
            self.state_label.configure(foreground='red')
        elif self.state == 'FINISHED':
            self.state_label.configure(foreground='blue')
        else:
            self.state_label.configure(foreground='black')

    def update_gui_once(self):
        self.canvas.delete('all')
        self.draw_grid()
        self.draw_path()
        self.draw_extra_map_layers()
        self.draw_other_vehicle()
        self.draw_vehicle()
        self.update_status_display()
        self.root.update_idletasks()

    def update_gui(self):
        self.canvas.delete('all')
        self.draw_grid()
        self.draw_path()
        self.draw_extra_map_layers()
        self.draw_other_vehicle()
        self.draw_vehicle()
        self.update_status_display()

        current_time = time.time()
        if self.follow_vehicle_var.get() and self.vehicle_position:
            if not self.initial_centering_done or current_time - self.initial_centering_time > 0.5:
                self.center_view()
                self.initial_centering_done = True
                self.initial_centering_time = current_time

        self.root.after(100, self.update_gui)


def main():
    rospy.init_node('path_gui')
    PathGuiNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
