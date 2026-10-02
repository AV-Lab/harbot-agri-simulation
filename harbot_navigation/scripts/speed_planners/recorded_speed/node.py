#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Read the selected route's speeds from the same JSON as the global planner.
# - Match the local planner's original point index to the recorded speed.
# - Publish zero speed when stopped, finished, or waiting for matching progress.
# - Publish the signed desired speed and the route's forward/reverse mode.
# ******************************************************************************


import rospy
import threading
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, UInt32
from recorded_route import load_recorded_route


class SpeedPlanner:
    def __init__(self):
        self.route = rospy.get_param("~route", "a16-a20")
        self.path_index_topic = rospy.get_param("~path_index_topic", "path_nav/path_index")
        self.local_path_topic = rospy.get_param("~local_path_topic", "path_nav/local_path")
        self.reverse_mode_topic = rospy.get_param("~reverse_mode_topic", "path_nav/reverse_mode")
        self.desired_speed_topic = rospy.get_param("~desired_speed_topic", "path_nav/desired_speed")
        self.movement_allowed_topic = rospy.get_param("~movement_allowed_topic", "path_nav/movement_allowed")

        self.path_index = None
        self.remaining_points = None
        self.movement_allowed = False
        self.state_lock = threading.Lock()
        self.speeds = [point[3] for point in load_recorded_route(self.route)]

        self.speed_pub = rospy.Publisher(self.desired_speed_topic, Float64, queue_size=1)
        self.reverse_mode_pub = rospy.Publisher(self.reverse_mode_topic, Bool, queue_size=1, latch=True)

        rospy.Subscriber(self.local_path_topic, Path, self.local_path_callback, queue_size=1)
        rospy.Subscriber(self.path_index_topic, UInt32, self.path_index_callback, queue_size=1)
        rospy.Subscriber(self.movement_allowed_topic, Bool, self.movement_allowed_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_speed)

    def path_index_callback(self, msg):
        with self.state_lock:
            self.path_index = msg.data

    def local_path_callback(self, msg):
        with self.state_lock:
            self.remaining_points = len(msg.poses)

    def movement_allowed_callback(self, msg):
        with self.state_lock:
            self.movement_allowed = msg.data
            if not self.movement_allowed:
                self.speed_pub.publish(Float64(data=0.0))

    def publish_speed(self, _event):
        with self.state_lock:
            route_speed = 0.0
            if (
                self.path_index is not None
                and 0 <= self.path_index < len(self.speeds)
                and self.remaining_points == len(self.speeds) - self.path_index
            ):
                route_speed = self.speeds[self.path_index]

            desired_speed = route_speed if self.movement_allowed else 0.0
            self.speed_pub.publish(Float64(data=desired_speed))
            self.reverse_mode_pub.publish(Bool(data=route_speed < 0.0))


def main():
    rospy.init_node("speed_planner")
    SpeedPlanner()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
