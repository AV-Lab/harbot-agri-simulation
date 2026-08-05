#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


ROBOT_NS = "/robot"
# ROBOT_NS = "/robot_b"


def franka_joint_names(robot_ns):
    prefix = "robot_b" if robot_ns.rstrip("/") == "/robot_b" else "robot"
    return [
        prefix + "_joint1",
        prefix + "_joint2",
        prefix + "_joint3",
        prefix + "_joint4",
        prefix + "_joint5",
        prefix + "_joint6",
        prefix + "_joint7",
    ]


def main():
    rospy.init_node("print_fr3_joint_values", anonymous=True)

    topic = ROBOT_NS.rstrip("/") + "/joint_states"
    msg = rospy.wait_for_message(topic, JointState, timeout=5.0)
    positions_by_name = dict(zip(msg.name, msg.position))
    joint_names = franka_joint_names(ROBOT_NS)

    missing = [name for name in joint_names if name not in positions_by_name]
    if missing:
        rospy.logerr("Missing joints on %s: %s", topic, ", ".join(missing))
        return 1

    joints = [positions_by_name[name] for name in joint_names]
    formatted = ", ".join("{:.3f}".format(value) for value in joints)
    print("self.current_joint_pos = [{}]".format(formatted))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
