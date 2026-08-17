"""ROS client for the simulator-independent harvested-object attachment API."""

import rospy

from harbot_control.srv import SetObjectAttachment, SetObjectAttachmentRequest
from harbot_harvest.helpers import resolve_ros_name


ATTACHMENT_TARGET_ENVIRONMENT = "environment"
ATTACHMENT_TARGET_GRIPPER = "gripper"
ATTACHMENT_TARGET_RELEASED = "released"
VALID_ATTACHMENT_TARGETS = frozenset(
    (
        ATTACHMENT_TARGET_ENVIRONMENT,
        ATTACHMENT_TARGET_GRIPPER,
        ATTACHMENT_TARGET_RELEASED,
    )
)


class ObjectAttachmentClient(object):
    """Ask the active attachment service to hold or release an object."""

    def __init__(self, robot_namespace):
        self.service_name = resolve_ros_name(
            robot_namespace,
            "object_attachment/set",
        )
        self._service = rospy.ServiceProxy(
            self.service_name,
            SetObjectAttachment,
        )

    def set_target(self, object_id, target):
        object_id = str(object_id).strip()
        target = str(target).strip().lower()

        if not object_id:
            raise ValueError("Attachment object_id cannot be empty")
        if target not in VALID_ATTACHMENT_TARGETS:
            raise ValueError(
                "Invalid attachment target '{}'; expected one of {}".format(
                    target,
                    ", ".join(sorted(VALID_ATTACHMENT_TARGETS)),
                )
            )

        request = SetObjectAttachmentRequest(object_id=object_id, target=target)

        try:
            rospy.wait_for_service(self.service_name, timeout=1.0)
            response = self._service(request)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("%s failed: %s", self.service_name, str(exc))
            return False

        if not response.success:
            rospy.logwarn(
                "%s rejected attachment target '%s' for '%s': %s",
                self.service_name,
                target,
                object_id,
                response.message,
            )
            return False

        return True
