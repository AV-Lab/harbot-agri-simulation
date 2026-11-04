# ******************************************************************************
# Code Summary:
# - Call the simulator-independent object attachment service of a robot.
# - Attach a harvested object to the gripper, or release it.
# - Return False and log a warning when the service fails or rejects a request.
# ******************************************************************************


import rospy
from harbot_harvest import helpers
from harbot_control.srv import SetObjectAttachment


ATTACHMENT_TARGET_GRIPPER = "gripper"
ATTACHMENT_TARGET_RELEASED = "released"


def set_attachment(robot_namespace, object_id, target):
    service_name = helpers.resolve_ros_name(robot_namespace, "object_attachment/set")
    service = rospy.ServiceProxy(service_name, SetObjectAttachment)

    try:
        rospy.wait_for_service(service_name, timeout=1.0)
        response = service(object_id, target)
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logwarn("%s failed: %s", service_name, exc)
        return False

    if not response.success:
        rospy.logwarn("%s rejected %s -> %s: %s", service_name, object_id, target, response.message)
    return response.success
