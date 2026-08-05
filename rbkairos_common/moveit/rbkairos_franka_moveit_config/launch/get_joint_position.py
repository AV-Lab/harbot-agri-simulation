#!/usr/bin/env python3
import sys

import rospy
from sensor_msgs.msg import JointState


def main():
    if len(sys.argv) < 3:
        sys.stderr.write("usage: get_joint_position.py JOINT_STATES_TOPIC JOINT_NAME [TIMEOUT]\n")
        return 2

    topic = sys.argv[1]
    joint = sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    rospy.init_node("get_joint_position", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    values = dict(zip(msg.name, msg.position))
    if joint not in values:
        sys.stderr.write("joint %s not found on %s\n" % (joint, topic))
        return 3

    print(repr(float(values[joint])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
