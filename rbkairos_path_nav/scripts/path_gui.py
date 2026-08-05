#!/usr/bin/env python3

import math
import time
import rospy
import threading
import tkinter as tk
from tkinter import ttk
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Bool, String
from tf.transformations import euler_from_quaternion


class PathGuiNode(object):
    def __init__(self):
        namespace = rospy.get_namespace().strip('/')
        self.robot_namespace = namespace or 'robot'
        self.is_robot_a = self.robot_namespace in ('robot', 'robot_a')
        self.other_robot_namespace = 'robot_b' if self.is_robot_a else 'robot'

        self.odom_topic = rospy.get_param('~odom_topic', 'robotnik_base_control/odom')
        self.other_odom_topic = rospy.get_param(
            '~other_odom_topic',
            '/{}/robotnik_base_control/odom_gt'.format(self.other_robot_namespace)
        )
        self.global_path_topic = rospy.get_param('~global_path_topic', 'path_nav/path')
        self.desired_velocity_topic = rospy.get_param('~desired_velocity_topic', 'path_nav/desired_speed')
        self.state_flag_topic = rospy.get_param('~state_flag_topic', 'path_nav/status')
        self.proceed_flag_topic = rospy.get_param('~proceed_flag_topic', 'path_nav/proceed_flag')
        self.reverse_mode_topic = rospy.get_param('~reverse_mode_topic', 'path_nav/reverse_mode')
        self.wheel_base = float(rospy.get_param('~wheel_base', 0.45))
        self.vehicle_width = float(rospy.get_param('~vehicle_width', 0.60))
        self.vehicle_length = float(rospy.get_param('~vehicle_length', 0.70))
        self.path_name = rospy.get_param('~path_name', 'example_straight.json')
        self.path_clearance_radius = float(rospy.get_param('~path_clearance_radius', 0.80))

        self.vehicle_position = [0.0, 0.0, 0.0]
        self.other_vehicle_position = []
        self.vehicle_velocity = 0.0
        self.path_points = []
        self.desired_speed = 0.0
        self.state = 'IDLE'
        self.proceed_flag = False
        self.scale = 200.0
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

    def start_gui(self):
        self.root = tk.Tk()
        namespace = rospy.get_namespace().strip('/') or 'global'
        self.root.title(f'ORBit Visualization - {namespace}')

        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        self.root.geometry(f'{screen_width}x{screen_height}+0+0')

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

        self.proceed_var = tk.StringVar(value='Proceed: False')
        ttk.Label(status_frame, textvariable=self.proceed_var).pack(anchor=tk.W)

        self.position_var = tk.StringVar(value='Position: (0.0, 0.0)')
        ttk.Label(status_frame, textvariable=self.position_var).pack(anchor=tk.W)

        self.other_position_var = tk.StringVar(value='Other: n/a')
        ttk.Label(status_frame, textvariable=self.other_position_var).pack(anchor=tk.W)

        self.path_info_var = tk.StringVar(value='Path Points: 0')
        ttk.Label(status_frame, textvariable=self.path_info_var).pack(anchor=tk.W)

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

        self.follow_vehicle_var = tk.BooleanVar(value=True)
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
        self.proceed_var.set(f'Proceed: {self.proceed_flag}')
        self.path_info_var.set(f'Path Points: {len(self.path_points)}')

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
        self.draw_other_vehicle()
        self.draw_vehicle()
        self.update_status_display()
        self.root.update_idletasks()

    def update_gui(self):
        self.canvas.delete('all')
        self.draw_grid()
        self.draw_path()
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
