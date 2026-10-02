#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Build the route file location from the robot namespace and route name.
# - Load path positions and orientations from the JSON file.
# - Convert the loaded points into a standard ROS Path message.
# - Publish the complete global path as a latched message.
# ******************************************************************************


import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from recorded_route import load_recorded_route
from tf.transformations import quaternion_from_euler


class PathLoader:
    def __init__(self):
        self.route = rospy.get_param("~route", "a16-a20")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.global_path_topic = rospy.get_param("~global_path_topic", "path_nav/global_path")

        self.global_path_pub = rospy.Publisher(self.global_path_topic, Path, queue_size=1, latch=True)

        try:
            route = self.load_path()
        except (OSError, TypeError, ValueError) as error:
            rospy.logerr("Could not load path %s: %s", self.route, error)
            return

        self.global_path_pub.publish(route)

        rospy.loginfo("Published %d path points from %s", len(route.poses), self.route)

    def load_path(self):
        points = load_recorded_route(self.route)
        stamp = rospy.Time.now()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        for x, y, yaw, _speed, _distance in points:
            orientation = quaternion_from_euler(0.0, 0.0, yaw)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
            path.poses.append(pose)

        return path


def main():
    rospy.init_node("path_loader")
    PathLoader()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
