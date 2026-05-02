#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64


class ContainerBottomCommand:
    def __init__(self):
        self.robot_ns = rospy.get_param("~robot_ns", "/robot")
        self.action = rospy.get_param("~action", "close").strip().lower()

        self.topic = self.robot_ns + "/container_bottom/container_bottom_controller/command"
        self.pub = rospy.Publisher(self.topic, Float64, queue_size=1, latch=True)

    def run(self):
        if self.action == "open":
            value = 0.7854
        elif self.action == "close":
            value = 0.0
        else:
            rospy.logerr("Invalid action: %s. Use open or close.", self.action)
            return

        rospy.sleep(1.0)
        self.pub.publish(Float64(data=value))
        rospy.loginfo("Sent %s command %.4f to %s", self.action, value, self.topic)
        rospy.sleep(1.0)


def main():
    rospy.init_node("container_control")
    ContainerBottomCommand().run()


if __name__ == "__main__":
    main()
