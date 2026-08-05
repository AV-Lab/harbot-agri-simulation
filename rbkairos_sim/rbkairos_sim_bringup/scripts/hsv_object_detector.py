#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
import time
import yaml
import os
from std_msgs.msg import String

class HSVObjectDetector:
    def __init__(self):
        rospy.init_node("hsv_object_detector", anonymous=True)
        rospy.sleep(3.0) 

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_position = None
        self.stable_start_time = None
        self.position_threshold = 0.002
        self.stable_duration = 1.0

        self.robot_ns = rospy.get_param('~robot_ns', '/robot')
        self.robot_name = self.robot_ns.strip('/')
        self.camera_name = self.robot_name + '_wrist_d435'
        self.camera_frame = self.camera_name + '_depth_optical_frame'

        # Get selected color from param
        self.color_name = rospy.get_param("~color_name", "orange")

        # Load HSV config from param
        self.color_ranges = self.load_color_config()

        # Publishers
        self.pub = rospy.Publisher(self.robot_ns + '/object_position', PointStamped, queue_size=1)
        self.size_pub = rospy.Publisher(self.robot_ns + '/object_size', PointStamped, queue_size=1)
        self.color_pub = rospy.Publisher(self.robot_ns + '/object_color', String, queue_size=1)


        # Camera topics
        self.camera_info_sub = rospy.Subscriber('/' + self.camera_name + '/color/camera_info', CameraInfo, self.camera_info_callback)
        color_sub = message_filters.Subscriber('/' + self.camera_name + '/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/' + self.camera_name + '/aligned_depth_to_color/image_raw', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        rospy.loginfo("🎯 Detecting objects of color: %s", self.color_name)
        rospy.spin()

    def load_color_config(self):
        config_path = rospy.get_param("~color_config", "/root/catkin_ws/src/harbot-agri-simulation/rbkairos_sim/rbkairos_sim_bringup/config/hsv_colors.yaml")
        if not config_path or not os.path.isfile(config_path):
            rospy.logwarn("⚠️ No valid color config file found, using default green range.")
            return {
                "lower": np.array([40, 70, 70], dtype=np.uint8),
                "upper": np.array([80, 255, 255], dtype=np.uint8)
            }

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        if self.color_name not in data:
            rospy.logwarn("⚠️ Color '%s' not found in config. Using default green.", self.color_name)
            return {
                "lower": np.array([40, 70, 70], dtype=np.uint8),
                "upper": np.array([80, 255, 255], dtype=np.uint8)
            }

        bounds = data[self.color_name]
        return {
            "lower": np.array(bounds["lower"], dtype=np.uint8),
            "upper": np.array(bounds["upper"], dtype=np.uint8)
        }

    def camera_info_callback(self, msg):
        self.camera_info = msg
        rospy.loginfo("📸 Received camera info")
        self.camera_info_sub.unregister()

    def image_callback(self, color_msg, depth_msg):
        if self.camera_info is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]

        mask = cv2.inRange(hsv, self.color_ranges["lower"], self.color_ranges["upper"])
        mask = cv2.erode(mask, None, 2)
        mask = cv2.dilate(mask, None, 2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = [c for c in contours if cv2.contourArea(c) >= 500]
        if not valid_contours:
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
            return

        c = max(valid_contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(c)
        cx_pixel = x + w // 2
        cy_pixel = y + h // 2

        if cx_pixel >= depth_image.shape[1] or cy_pixel >= depth_image.shape[0]:
            return

        Z = depth_image[cy_pixel, cx_pixel] / 1000.0
        if Z <= 0 or np.isnan(Z):
            return

        # 1. Get pixel-based width/height first
        width_m = (w * Z) / fx
        height_m = (h * Z) / fy

        # 2. Estimate object thickness from floor
        floor_y = min(cy_pixel + int(0.5 * h), depth_image.shape[0] - 1)
        Z_floor = depth_image[floor_y, cx_pixel] / 1000.0

        thickness_z = Z_floor - Z
        if thickness_z < 0 or np.isnan(thickness_z):
            thickness_z = 0.5 * ((width_m + height_m) / 2.0)  # fallback

        # 3. Deproject 3D point
        X = (cx_pixel - cx) * Z / fx
        Y = (cy_pixel - cy) * Z / fy
        position = np.array([X, Y, Z])

        if self.last_position is not None:
            distance = np.linalg.norm(self.last_position - position)
            if distance < self.position_threshold:
                if self.stable_start_time is None:
                    self.stable_start_time = time.time()
                elif time.time() - self.stable_start_time >= self.stable_duration:
                    point = PointStamped()
                    point.header.stamp = rospy.Time.now()
                    point.header.frame_id = self.camera_frame
                    point.point.x = X
                    point.point.y = Y
                    point.point.z = Z
                    self.pub.publish(point)

                    size_msg = PointStamped()
                    size_msg.header = point.header
                    size_msg.point.x = height_m 
                    size_msg.point.y = width_m
                    size_msg.point.z = thickness_z

                    self.size_pub.publish(size_msg)
                    self.color_pub.publish(String(self.color_name))


                    rospy.loginfo("✅ [%s] Position: %.3f %.3f %.3f | Size: %.3f x %.3f x %.3f m",
                                self.color_name, X, Y, Z, width_m, height_m, thickness_z)

                    self.stable_start_time = None
            else:
                self.stable_start_time = None

        self.last_position = position

        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(frame, (cx_pixel, cy_pixel), 8, (255, 0, 255), -1)
        cv2.circle(frame, (int(cx), int(cy)), 6, (255, 255, 255), 2)
        cv2.putText(frame, f"{self.color_name}: {Z:.2f}m", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # cv2.imshow("Mask", mask)
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        HSVObjectDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
