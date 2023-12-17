#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorTrackingNode(Node):
    def __init__(self):
        super().__init__("color_tracking_node")
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.depth_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/limo/depth_camera_link/depth/camera_info",
            self.camera_info_callback,
            10
        )
        self.cv_image = None
        self.depth_image = None
        self.camera_info = None

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def get_depth(self, x, y):
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        depth = self.depth_image[y, x]
        depth = depth * self.camera_info.depth_scale

        return depth

    def track_colors(self):
        if self.cv_image is None or self.depth_image is None or self.camera_info is None:
            return

        # Apply necessary pre-processing steps (e.g., resizing, noise reduction, enhancement)

        # Convert image to the desired color space
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for detection (example: red, green, blue)
        color_ranges = [
            ((150, 50, 50), (170, 255, 255))]

        color_counts = [0] * len(color_ranges)
        color_positions = [[] for _ in range(len(color_ranges))]
        color_distances = [[] for _ in range(len(color_ranges))]

        for i, (lower, upper) in enumerate(color_ranges):
            mask = cv2.inRange(hsv_image, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    depth = self.get_depth(cx, cy)

                    color_counts[i] += 1
                    color_positions[i].append((cx, cy))
                    color_distances[i].append(depth)

        # Print the color counts, positions, and distances
        for i, count in enumerate(color_counts):
            color_name = ["Potholes"][i]  # Replace with your color names
            positions = color_positions[i]
            distances = color_distances[i]

            self.get_logger().info(f"{color_name} count: {count}")
            for j, (x, y) in enumerate(positions):
                distance = distances[j]
                self.get_logger().info(f"Location {j+1}: (x={x}, y={y}), Distance={distance}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()