#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ROBOT_NS = "/robot"
ROBOT_NS = "/robot_b"

## TOP POSES
POSE_VALUE = [-0.249, -0.914, -0.461, -2.035, 1.494, 0.906, -0.273]  # looking back, arm right

# POSE_VALUE = [-0.241, -1.294, -0.601, -1.975, 1.576, 0.852, 0.225] # looking front, arm right
# POSE_VALUE = [-0.125, 0.653, 0.982, -0.622, -2.408, 1.137, 1.831] # looking back, arm left
# POSE_VALUE = [-0.734, -0.632, 0.671, -1.777, -1.178, 1.575, 2.058] # looking front, arm left

## BOTTOM POSES
# POSE_VALUE = [-0.429, 0.233, 0.030, -1.904, 1.325, 1.211, -1.360] # looking back, arm right
# POSE_VALUE = [-0.510, -0.155, -0.050, -2.357, 1.190, 1.105, -1.470] # looking front, arm right
# POSE_VALUE = [0.163, -1.037, 0.807, -2.724, -0.755, 1.074, 2.606] # looking back, arm left
# POSE_VALUE = [-2.086, -1.062, -1.309, -2.582, -2.480, 4.206, 2.007] # looking front, arm left


def franka_joint_names(robot_ns):
    prefix = "robot_b" if robot_ns.rstrip("/") == "/robot_b" else "robot"
    return ["{}_joint{}".format(prefix, index) for index in range(1, 8)]


def build_joint_command(joint_names, positions, duration):
    command = JointTrajectory()
    command.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)
    command.points = [point]
    return command


def main():
    rospy.init_node("move_franka_to_joint_pose", anonymous=True)

    if len(POSE_VALUE) != 7:
        rospy.logerr("POSE_VALUE must contain 7 joint values.")
        return 1

    topic = ROBOT_NS.rstrip("/") + "/arm/pos_traj_controller/command"
    publisher = rospy.Publisher(topic, JointTrajectory, queue_size=1)
    rospy.sleep(0.5)

    publisher.publish(build_joint_command(franka_joint_names(ROBOT_NS), POSE_VALUE, 3.0))
    rospy.loginfo("Published Franka arm joint pose to %s", topic)
    rospy.sleep(0.2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
