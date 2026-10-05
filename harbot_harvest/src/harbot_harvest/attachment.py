# ******************************************************************************
# Code Summary:
# - Call the simulator-independent object attachment service of a robot.
# - Attach a harvested object to the environment or the gripper, or release it.
# - Return False and log a warning when the service fails or rejects a request.
# ******************************************************************************


import rospy
from harbot_harvest.helpers import resolve_ros_name
from harbot_control.srv import SetObjectAttachment, SetObjectAttachmentRequest


ATTACHMENT_TARGET_GRIPPER = "gripper"
ATTACHMENT_TARGET_RELEASED = "released"
ATTACHMENT_TARGET_ENVIRONMENT = "environment"
VALID_ATTACHMENT_TARGETS = (ATTACHMENT_TARGET_ENVIRONMENT, ATTACHMENT_TARGET_GRIPPER, ATTACHMENT_TARGET_RELEASED)


class ObjectAttachmentClient:
    def __init__(self, robot_namespace):
        self.service_name = resolve_ros_name(robot_namespace, "object_attachment/set")
        self.service = rospy.ServiceProxy(self.service_name, SetObjectAttachment)

    def set_target(self, object_id, target):
        object_id = str(object_id).strip()
        target = str(target).strip().lower()

        if not object_id:
            raise ValueError("Attachment object_id cannot be empty")
        if target not in VALID_ATTACHMENT_TARGETS:
            raise ValueError("Invalid attachment target '{}'; expected one of {}".format(target, ", ".join(sorted(VALID_ATTACHMENT_TARGETS))))

        try:
            rospy.wait_for_service(self.service_name, timeout=1.0)
            response = self.service(SetObjectAttachmentRequest(object_id=object_id, target=target))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("%s failed: %s", self.service_name, exc)
            return False

        if not response.success:
            rospy.logwarn("%s rejected attachment target '%s' for '%s': %s", self.service_name, target, object_id, response.message)
            return False

        return True
