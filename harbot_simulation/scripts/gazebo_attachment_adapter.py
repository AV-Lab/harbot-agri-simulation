#!/usr/bin/env python3

import threading

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from greenhouse_fruit_detachment.srv import DetachFruit
from harbot_control.srv import SetObjectAttachment, SetObjectAttachmentResponse


TARGET_ENVIRONMENT = "environment"
TARGET_GRIPPER = "gripper"
TARGET_RELEASED = "released"
HOLDER_TARGETS = frozenset((TARGET_ENVIRONMENT, TARGET_GRIPPER))
VALID_TARGETS = HOLDER_TARGETS | frozenset((TARGET_RELEASED,))


class GazeboAttachmentAdapter:
    """Handle semantic object attachment requests using the Gazebo plugins.

    Fruits hang on the plants through the fruit detachment plugin. A fruit's
    stem only breaks when it is attached to the gripper, so a failed grasp
    leaves the fruit on its plant.
    """

    def __init__(self):
        self.robot_model = rospy.get_param("~robot_model")
        self.object_link = rospy.get_param("~object_link", "body")
        self.holder_links = {
            TARGET_ENVIRONMENT: rospy.get_param("~base_link"),
            TARGET_GRIPPER: rospy.get_param("~gripper_link"),
        }
        self.attach_service_name = rospy.get_param(
            "~gazebo_attach_service", "/link_attacher_node/attach"
        )
        self.detach_service_name = rospy.get_param(
            "~gazebo_detach_service", "/link_attacher_node/detach"
        )
        self.stem_service_name = rospy.get_param(
            "~fruit_detach_service", "/fruit_detachment/detach"
        )
        self.wait_timeout = float(rospy.get_param("~service_wait_timeout", 1.0))
        self.attempts = max(1, int(rospy.get_param("~service_attempts", 5)))
        self.retry_delay = float(rospy.get_param("~service_retry_delay", 0.1))
        self._states = {}
        self._lock = threading.Lock()
        self._service = rospy.Service(
            "object_attachment/set",
            SetObjectAttachment,
            self._handle_request,
        )
        rospy.loginfo(
            "Gazebo object attachment adapter ready at %s",
            self._service.resolved_name,
        )

    def _handle_request(self, request):
        with self._lock:
            success, message = self.set_target(request.object_id, request.target)

        log = rospy.loginfo if success else rospy.logwarn
        log("Object attachment request: %s", message)
        return SetObjectAttachmentResponse(success=success, message=message)

    def set_target(self, object_id, target):
        object_id = object_id.strip()
        target = target.strip().lower()

        if not object_id:
            return False, "object_id must not be empty"
        if target not in VALID_TARGETS:
            return False, "unsupported attachment target '{}'".format(target)

        current = self._states.get(object_id)
        if current == target:
            return True, "{} is already {}".format(object_id, target)

        if target == TARGET_RELEASED:
            return self._release(object_id, current)
        return self._move_to_holder(object_id, current, target)

    def _move_to_holder(self, object_id, current, target):
        if not self._attach(object_id, target):
            return False, "failed to attach {} to {}".format(object_id, target)

        if target == TARGET_GRIPPER and not self._break_stem(object_id):
            self._detach(object_id, target)
            return False, "failed to break the stem of {}".format(object_id)

        previous = current if current in HOLDER_TARGETS else None
        if current is None and target == TARGET_GRIPPER:
            previous = TARGET_ENVIRONMENT

        if previous is not None and not self._detach(object_id, previous):
            rolled_back = self._detach(object_id, target)
            rollback_note = "rollback succeeded" if rolled_back else "rollback failed"
            return False, "failed to detach {} from {}; {}".format(
                object_id,
                previous,
                rollback_note,
            )

        self._states[object_id] = target
        return True, "attached {} to {}".format(object_id, target)

    def _release(self, object_id, current):
        if current in HOLDER_TARGETS:
            if not self._detach(object_id, current):
                return False, "failed to release {} from {}".format(object_id, current)
        elif current is None:
            detached_from_gripper = self._detach(object_id, TARGET_GRIPPER)
            detached_from_environment = self._detach(object_id, TARGET_ENVIRONMENT)
            if not detached_from_gripper and not detached_from_environment:
                return False, "could not verify that {} is released".format(object_id)

        self._states[object_id] = TARGET_RELEASED
        return True, "released {}".format(object_id)

    def _break_stem(self, object_id):
        try:
            rospy.wait_for_service(self.stem_service_name, timeout=self.wait_timeout)
            response = rospy.ServiceProxy(self.stem_service_name, DetachFruit)(object_id)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("%s failed for %s: %s", self.stem_service_name, object_id, str(exc))
            return False

        if not response.success:
            rospy.logwarn("%s rejected %s: %s", self.stem_service_name, object_id, response.message)
        return response.success

    def _attach(self, object_id, holder):
        return self._call_plugin(self.attach_service_name, object_id, holder)

    def _detach(self, object_id, holder):
        return self._call_plugin(self.detach_service_name, object_id, holder)

    def _call_plugin(self, service_name, object_id, holder):
        request = AttachRequest()
        request.model_name_1 = self.robot_model
        request.link_name_1 = self.holder_links[holder]
        request.model_name_2 = object_id
        request.link_name_2 = self.object_link

        for attempt in range(1, self.attempts + 1):
            try:
                rospy.wait_for_service(service_name, timeout=self.wait_timeout)
                response = rospy.ServiceProxy(service_name, Attach)(request)
                if response.ok:
                    rospy.sleep(self.retry_delay)
                    return True
                rospy.logwarn(
                    "%s returned ok=False for %s -> %s (%d/%d)",
                    service_name,
                    object_id,
                    holder,
                    attempt,
                    self.attempts,
                )
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn(
                    "%s failed for %s -> %s (%d/%d): %s",
                    service_name,
                    object_id,
                    holder,
                    attempt,
                    self.attempts,
                    str(exc),
                )

            if attempt < self.attempts:
                rospy.sleep(self.retry_delay)

        return False


def main():
    rospy.init_node("gazebo_attachment_adapter")
    GazeboAttachmentAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
