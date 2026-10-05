# ******************************************************************************
# Code Summary:
# - Build PoseStamped and Marker messages from plain position tuples.
# - Track the latest message on a topic and wait for one newer than a time.
# - Start ROS commands in their own process group and shut them down cleanly.
# ******************************************************************************


import os
import time
import rospy
import signal
import subprocess
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped


IDENTITY_ORIENTATION = (0.0, 0.0, 0.0, 1.0)


def set_pose(pose, position, orientation=IDENTITY_ORIENTATION):
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation


def make_pose_stamped(frame_id, position, orientation=IDENTITY_ORIENTATION):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    set_pose(pose.pose, position, orientation)
    return pose


def make_marker(frame_id, namespace, marker_id, marker_type, position, scale, color, orientation=IDENTITY_ORIENTATION):
    marker = Marker()
    marker.id = marker_id
    marker.ns = namespace
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.header.frame_id = frame_id
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    set_pose(marker.pose, position, orientation)
    return marker


class LatestMessage:
    def __init__(self, topic, message_type):
        self.latest = None
        self.subscriber = rospy.Subscriber(topic, message_type, self.callback, queue_size=10)

    def callback(self, msg):
        self.latest = (msg.data, time.monotonic())

    def value_since(self, start_time):
        latest = self.latest
        if latest is not None and latest[1] >= start_time:
            return latest[0]
        return None

    def wait_since(self, start_time, timeout=1.0, default=0.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown():
            value = self.value_since(start_time)
            if value is not None:
                return value
            if time.monotonic() >= deadline:
                break
            rospy.sleep(0.05)
        return default


def rosrun_command(package_name, script_name, private_params):
    command = ["rosrun", package_name, script_name]
    command.extend("_{}:={}".format(name, value) for name, value in private_params.items())
    return command


def start_process(command, quiet=False):
    output = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(command, preexec_fn=os.setsid, stdout=output, stderr=output)


def shutdown_process(process_handle, timeout=5.0):
    if process_handle is None or process_handle.poll() is not None:
        return

    try:
        process_group = os.getpgid(process_handle.pid)
        os.killpg(process_group, signal.SIGINT)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if process_handle.poll() is not None:
                return
            time.sleep(0.1)

        os.killpg(process_group, signal.SIGTERM)
    except ProcessLookupError:
        pass
