#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Offer the simulator-independent object_attachment/set service for one robot.
# - gripper: break the orange's stem on the plant, then attach it to the gripper.
# - released: detach the orange from the gripper, so it drops.
# - If the attach fails after the stem breaks, the orange drops.
# - Retry the link attacher a few times before reporting a failure.
# ******************************************************************************


import rospy
import threading
from gazebo_ros_link_attacher.srv import Attach
from greenhouse_fruit_detachment.srv import DetachFruit
from harbot_control.srv import SetObjectAttachment, SetObjectAttachmentResponse


TARGET_GRIPPER = "gripper"
TARGET_RELEASED = "released"


class GazeboAttachmentAdapter:
    def __init__(self):
        self.robot_model = rospy.get_param("~robot_model")
        self.gripper_link = rospy.get_param("~gripper_link")
        self.object_link = rospy.get_param("~object_link", "body")

        self.states = {}
        self.attempts = 5
        self.retry_delay = 0.1
        self.wait_timeout = 1.0
        self.lock = threading.Lock()

        self.attach_service = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.detach_service = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        self.stem_service = rospy.ServiceProxy("/fruit_detachment/detach", DetachFruit)
        self.service = rospy.Service("object_attachment/set", SetObjectAttachment, self.handle_request)

        rospy.loginfo("Gazebo object attachment adapter ready at %s", self.service.resolved_name)

    def handle_request(self, request):
        with self.lock:
            success, message = self.set_target(request.object_id, request.target)

        log = rospy.loginfo if success else rospy.logwarn
        log("Object attachment request: %s", message)
        return SetObjectAttachmentResponse(success, message)

    def set_target(self, object_id, target):
        if not object_id or target not in (TARGET_GRIPPER, TARGET_RELEASED):
            return False, "Invalid request: '{}' -> '{}'.".format(object_id, target)
        if self.states.get(object_id) == target:
            return True, "{} is already {}.".format(object_id, target)

        if target == TARGET_RELEASED:
            if not self.call_link_attacher(self.detach_service, object_id):
                return False, "Failed to release {}.".format(object_id)
        else:
            if not self.break_stem(object_id):
                return False, "Failed to break the stem of {}.".format(object_id)
            if not self.call_link_attacher(self.attach_service, object_id):
                return False, "Failed to attach {} to the gripper.".format(object_id)

        self.states[object_id] = target
        return True, "Set {} to {}.".format(object_id, target)

    def break_stem(self, object_id):
        try:
            self.stem_service.wait_for_service(self.wait_timeout)
            response = self.stem_service(object_id)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("%s failed for %s: %s", self.stem_service.resolved_name, object_id, exc)
            return False

        if not response.success:
            rospy.logwarn("%s rejected %s: %s", self.stem_service.resolved_name, object_id, response.message)
        return response.success

    def call_link_attacher(self, service, object_id):
        for _ in range(self.attempts):
            try:
                service.wait_for_service(self.wait_timeout)
                if service(self.robot_model, self.gripper_link, object_id, self.object_link).ok:
                    # Give Gazebo a moment to apply the joint change.
                    rospy.sleep(self.retry_delay)
                    return True
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("%s failed for %s: %s", service.resolved_name, object_id, exc)
            rospy.sleep(self.retry_delay)

        return False


def main():
    rospy.init_node("gazebo_attachment_adapter")
    GazeboAttachmentAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
