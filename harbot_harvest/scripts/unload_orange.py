#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Move the arm holding the orange to the drop pose above the container.
# - Release the orange through the attachment service and open the gripper.
# - Move the arm back to its home pose.
# - Publish orange_action with the unload result.
# ******************************************************************************


import rospy
from std_msgs.msg import Bool
from harbot_harvest import arm, helpers
from trajectory_msgs.msg import JointTrajectory
from harbot_harvest.attachment import ATTACHMENT_TARGET_RELEASED, ObjectAttachmentClient


class OrangeUnloader:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.arm_type = rospy.get_param("~arm", "Franka")
        self.orange_model = rospy.get_param("~orange", "").strip()
        self.result_topic = helpers.get_robot_topic(self.robot_ns, "result_topic")
        self.arm_cmd_topic = helpers.get_robot_topic(self.robot_ns, "arm_command_topic")
        self.gripper_cmd_topic = helpers.get_robot_topic(self.robot_ns, "gripper_command_topic")

        self.arm = arm.get_arm_config(self.arm_type, self.robot_ns)
        self.attachment_client = ObjectAttachmentClient(self.robot_ns)

        self.result_pub = rospy.Publisher(self.result_topic, Bool, queue_size=1)
        self.arm_pub = rospy.Publisher(self.arm_cmd_topic, JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher(self.gripper_cmd_topic, JointTrajectory, queue_size=1)

    def publish_result(self, value):
        self.result_pub.publish(Bool(data=value))

    def run(self):
        if not self.orange_model:
            rospy.logerr("Parameter ~orange is required")
            return
        if self.arm.drop_pose is None:
            rospy.logerr("No drop pose is configured for the %s arm", self.arm_type)
            self.publish_result(False)
            return

        rospy.sleep(0.5)
        rospy.loginfo("Moving orange to drop position")
        self.arm_pub.publish(arm.joint_command(self.arm.arm_joints, self.arm.drop_pose, 3.0))
        rospy.sleep(4.0)

        if not self.attachment_client.set_target(self.orange_model, ATTACHMENT_TARGET_RELEASED):
            rospy.logerr("Could not release %s through the attachment service", self.orange_model)
            self.publish_result(False)
            return

        rospy.sleep(0.1)
        rospy.loginfo("Unloading orange")
        self.gripper_pub.publish(arm.joint_command([self.arm.gripper_joint], [self.arm.gripper_open], 2.0))
        rospy.sleep(0.5)

        rospy.loginfo("Unloaded orange. Returning to initial position")
        self.arm_pub.publish(arm.joint_command(self.arm.arm_joints, self.arm.home_pose, 3.0))
        rospy.sleep(0.5)

        rospy.loginfo("Arm returned to initial position")
        self.publish_result(True)


def main():
    rospy.init_node("unload_orange", anonymous=True)
    OrangeUnloader().run()


if __name__ == "__main__":
    main()
