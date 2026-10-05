#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Receive synchronized color and aligned depth images from the wrist camera.
# - Find the largest orange-colored blob with an HSV threshold.
# - Deproject its center into a 3D point in the camera depth frame.
# - Estimate the object width, height, and thickness from the depth image.
# - Publish position, size, and color once the point is stable for 1 second.
# ******************************************************************************


import cv2
import time
import rospy
import numpy as np
import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import String
from harbot_harvest import helpers
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image


HSV_LOWER = np.array([5, 150, 50], dtype=np.uint8)
HSV_UPPER = np.array([25, 255, 240], dtype=np.uint8)
MIN_CONTOUR_AREA = 500


class HSVObjectDetector:
    def __init__(self):
        self.robot_ns = helpers.normalize_robot_namespace(rospy.get_param("~robot_ns", "/robot"))

        self.color_name = rospy.get_param("~color_name", "orange")
        self.camera_name = self.robot_ns.strip("/") + "_wrist_d435"
        self.camera_frame = self.camera_name + "_depth_optical_frame"

        self.camera_info = None
        self.bridge = CvBridge()
        self.last_position = None
        self.stable_duration = 1.0
        self.stable_start_time = None
        self.position_threshold = 0.002

        self.color_pub = rospy.Publisher(self.robot_ns + "/object_color", String, queue_size=1)
        self.size_pub = rospy.Publisher(self.robot_ns + "/object_size", PointStamped, queue_size=1)
        self.position_pub = rospy.Publisher(self.robot_ns + "/object_position", PointStamped, queue_size=1)

        color_sub = message_filters.Subscriber("/" + self.camera_name + "/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/" + self.camera_name + "/aligned_depth_to_color/image_raw", Image)
        self.camera_info_sub = rospy.Subscriber("/" + self.camera_name + "/color/camera_info", CameraInfo, self.camera_info_callback)

        self.image_sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.image_sync.registerCallback(self.image_callback)

        rospy.loginfo("Detecting objects of color: %s", self.color_name)

    def camera_info_callback(self, msg):
        self.camera_info = msg
        rospy.loginfo("Received camera info")
        self.camera_info_sub.unregister()

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
        width_m = (w * z) / fx
        height_m = (h * z) / fy

        # Thickness is the depth step to the background just below the object.
        floor_y = min(cy_pixel + int(0.5 * h), depth_image.shape[0] - 1)
        thickness_z = depth_image[floor_y, cx_pixel] / 1000.0 - z
        if thickness_z < 0 or np.isnan(thickness_z):
            thickness_z = 0.5 * ((width_m + height_m) / 2.0)

        position = np.array([(cx_pixel - cx) * z / fx, (cy_pixel - cy) * z / fy, z])
        if self.last_position is not None:
            if np.linalg.norm(self.last_position - position) >= self.position_threshold:
                self.stable_start_time = None
            elif self.stable_start_time is None:
                self.stable_start_time = time.time()
            elif time.time() - self.stable_start_time >= self.stable_duration:
                self.publish_detection(position, (height_m, width_m, thickness_z))
                self.stable_start_time = None

        self.last_position = position

    def publish_detection(self, position, size):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.camera_frame
        point.point.x, point.point.y, point.point.z = position
        self.position_pub.publish(point)

        size_msg = PointStamped()
        size_msg.header = point.header
        size_msg.point.x, size_msg.point.y, size_msg.point.z = size
        self.size_pub.publish(size_msg)
        self.color_pub.publish(String(self.color_name))


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
