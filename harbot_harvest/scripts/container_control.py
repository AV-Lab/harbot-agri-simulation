#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Read the requested container action, open or close.
# - Publish the matching container bottom joint position once.
# - Exit with an error code when the action is unknown.
# ******************************************************************************


import sys
import rospy
from std_msgs.msg import Float64
from harbot_harvest import helpers


CONTAINER_POSITIONS = {
    "open": 0.7854,
    "close": 0.0,
}


class ContainerBottomCommand:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.action = str(rospy.get_param("~action", "close")).strip().lower()
        self.topic = helpers.resolve_ros_name(self.robot_ns, "container_bottom/container_bottom_controller/command")

        self.container_pub = rospy.Publisher(self.topic, Float64, queue_size=1, latch=True)

    def run(self):
        if self.action not in CONTAINER_POSITIONS:
            rospy.logerr("Invalid action: %s. Use open or close.", self.action)
            return False

        rospy.sleep(0.5)
        self.container_pub.publish(Float64(data=CONTAINER_POSITIONS[self.action]))
        rospy.loginfo("Container %s", "opened" if self.action == "open" else "closed")
        rospy.sleep(0.5)
        return True


def main():
    rospy.init_node("container_control", anonymous=True)
    return 0 if ContainerBottomCommand().run() else 1


if __name__ == "__main__":
    sys.exit(main())
