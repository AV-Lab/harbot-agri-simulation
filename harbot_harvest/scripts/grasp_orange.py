#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Decide with the real-world grasp success probability if the grasp works.
# - On success, attach the orange to the gripper and close the gripper.
# - Move the arm back to its home pose with MoveIt, retrying until it works.
# - On failure, return the arm home and release the orange.
# - Publish orange_action with the grasp result.
# ******************************************************************************


import sys
import rospy
import random
import moveit_commander
from std_msgs.msg import Bool
from harbot_harvest import arm, helpers
from trajectory_msgs.msg import JointTrajectory
from harbot_harvest.attachment import ATTACHMENT_TARGET_GRIPPER, ATTACHMENT_TARGET_RELEASED, ObjectAttachmentClient


class OrangeGrasper:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.orange_model = rospy.get_param("~orange", "").strip()
        self.result_topic = helpers.get_robot_topic(self.robot_ns, "result_topic")
        self.return_motion_retries = int(rospy.get_param("~return_motion_retries", -1))
        self.return_planning_time = float(rospy.get_param("~return_planning_time", 10.0))
        self.return_planning_attempts = int(rospy.get_param("~return_planning_attempts", 10))
        self.move_group_wait_timeout = float(rospy.get_param("~move_group_wait_timeout", 50.0))
        self.gripper_cmd_topic = helpers.get_robot_topic(self.robot_ns, "gripper_command_topic")
        self.return_motion_retry_delay = float(rospy.get_param("~return_motion_retry_delay", 0.5))
        self.grasp_success_probability = float(rospy.get_param("~grasp_success_probability", 0.91))

        self.arm = arm.get_arm_config(self.robot_ns)
        self.attachment_client = ObjectAttachmentClient(self.robot_ns)

        self.result_pub = rospy.Publisher(self.result_topic, Bool, queue_size=1)
        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)

    def simulated_grasp_succeeds(self):
        # 0.91 is the grasp success rate measured in real-world experiments.
        if not 0.0 <= self.grasp_success_probability <= 1.0:
            rospy.logwarn("Invalid grasp_success_probability. Using 0.8")
            self.grasp_success_probability = 0.8

        return random.random() < self.grasp_success_probability

    def move_arm_home(self):
        moveit_commander.roscpp_initialize(sys.argv)
        group = None
        attempt = 1

        try:
            while not rospy.is_shutdown():
                try:
                    if group is None:
                        group = arm.create_move_group(self.arm.move_group, self.robot_ns, self.move_group_wait_timeout)
                        group.set_planning_time(self.return_planning_time)
                        group.set_num_planning_attempts(self.return_planning_attempts)
                        group.set_max_acceleration_scaling_factor(0.6)
                        group.set_max_velocity_scaling_factor(0.4)

                    group.set_start_state_to_current_state()
                    group.set_joint_value_target(dict(zip(self.arm.arm_joints, self.arm.home_pose)))

                    ok = group.go(wait=True)
                    group.stop()
                    group.clear_pose_targets()
                    if ok:
                        return True
                except Exception:
                    if group is not None:
                        try:
                            group.stop()
                            group.clear_pose_targets()
                        except Exception:
                            pass
                    group = None

                if self.return_motion_retries >= 0 and attempt >= self.return_motion_retries:
                    return False

                attempt += 1
                rospy.sleep(self.return_motion_retry_delay)

            return False
        finally:
            moveit_commander.roscpp_shutdown()

    def publish_result(self, value):
        self.result_pub.publish(Bool(data=value))

    def run(self):
        if not self.simulated_grasp_succeeds():
            rospy.logwarn("Grasp attempt failed for %s (simulated success probability %.2f).", self.orange_model, self.grasp_success_probability)
            if self.move_arm_home():
                rospy.sleep(1.0)
                if self.attachment_client.set_target(self.orange_model, ATTACHMENT_TARGET_RELEASED):
                    rospy.sleep(0.5)
            self.publish_result(False)
            return

        rospy.sleep(0.1)
        if not self.attachment_client.set_target(self.orange_model, ATTACHMENT_TARGET_GRIPPER):
            self.publish_result(False)
            return

        rospy.sleep(0.5)
        rospy.loginfo("Grasping orange")
        self.gripper_pub.publish(arm.joint_command([self.arm.gripper_joint], [self.arm.gripper_close], 2.0))
        rospy.sleep(2.0)

        rospy.loginfo("Grasped orange. Moving to unload position using MoveIt")
        if not self.move_arm_home():
            self.publish_result(False)
            return

        rospy.loginfo("Arm moved to holding position")
        self.publish_result(True)


def main():
    rospy.init_node("grasp_orange", anonymous=True)
    OrangeGrasper().run()


if __name__ == "__main__":
    main()
