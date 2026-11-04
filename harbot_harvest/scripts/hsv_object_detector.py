#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Receive synchronized color and aligned depth images from the wrist camera.
# - Find the largest orange-colored blob with an HSV threshold.
# - Deproject its center into a 3D point in the camera depth frame.
# - Publish position once the point is stable for 1 second.
# ******************************************************************************


import cv2
import time
import rospy
import numpy as np
import message_filters
from cv_bridge import CvBridge
from harbot_harvest import helpers
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image


HSV_LOWER = np.array([5, 150, 50], dtype=np.uint8)
HSV_UPPER = np.array([25, 255, 240], dtype=np.uint8)
MIN_CONTOUR_AREA = 500


class HSVObjectDetector:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.camera_name = self.robot_ns.strip("/") + "_wrist_d435"
        self.camera_frame = self.camera_name + "_depth_optical_frame"

        self.camera_info = None
        self.bridge = CvBridge()
        self.last_position = None
        self.stable_duration = 1.0
        self.stable_start_time = None
        self.position_threshold = 0.002

        self.position_pub = rospy.Publisher(self.robot_ns + "/object_position", PointStamped, queue_size=1)

        self.color_sub = message_filters.Subscriber("/" + self.camera_name + "/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/" + self.camera_name + "/aligned_depth_to_color/image_raw", Image)
        self.camera_info_sub = rospy.Subscriber("/" + self.camera_name + "/color/camera_info", CameraInfo, self.camera_info_callback)

        self.image_sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.image_sync.registerCallback(self.image_callback)

    def camera_info_callback(self, msg):
        if self.camera_info is None or msg.K != self.camera_info.K:
            self.camera_info = msg
            rospy.loginfo("Received camera info")

    def image_callback(self, color_msg, depth_msg):
        if self.camera_info is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logerr("cv_bridge error: %s", exc)
            return

        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), HSV_LOWER, HSV_UPPER)
        mask = cv2.dilate(cv2.erode(mask, None, 2), None, 2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if cv2.contourArea(contour) >= MIN_CONTOUR_AREA]
        if not contours:
            return

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        cx_pixel = x + w // 2
        cy_pixel = y + h // 2
        if cx_pixel >= depth_image.shape[1] or cy_pixel >= depth_image.shape[0]:
            return

        z = depth_image[cy_pixel, cx_pixel] / 1000.0
        if z <= 0 or np.isnan(z):
            return

        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]

        position = np.array([(cx_pixel - cx) * z / fx, (cy_pixel - cy) * z / fy, z])
        if self.last_position is not None:
            if np.linalg.norm(self.last_position - position) >= self.position_threshold:
                self.stable_start_time = None
            elif self.stable_start_time is None:
                self.stable_start_time = time.time()
            elif time.time() - self.stable_start_time >= self.stable_duration:
                self.publish_detection(position)
                self.stable_start_time = None

        self.last_position = position

    def publish_detection(self, position):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.camera_frame
        point.point.x, point.point.y, point.point.z = position
        self.position_pub.publish(point)


def main():
    rospy.init_node("hsv_object_detector", anonymous=True)
    rospy.sleep(0.1)
    HSVObjectDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
