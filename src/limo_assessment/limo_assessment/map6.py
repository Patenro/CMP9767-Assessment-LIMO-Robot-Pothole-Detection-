#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        self.color_subscriber = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.color_callback,
            10
        )
        self.depth_subscriber = self.create_subscription(
            Image,
            '/limo/depth_camera_link/depth/image_raw',
            self.depth_callback,
            10
        )
        self.depth_image = None

    def color_callback(self, color_msg):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        # Convert BGR image to HSV color space
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for pink color
        lower_pink = np.array([150, 50, 50], dtype=np.uint8)
        upper_pink = np.array([170, 255, 255], dtype=np.uint8)

        # Create a binary mask of pixels that fall within the color range
        mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

        # Find contours of the color regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Calculate the centroid of the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Get the depth information at the centroid position
            depth = self.get_depth(cx, cy)

            # Draw a circle at the centroid position and put the distance information on the image
            cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f'Distance: {depth:.2f} meters', (cx - 50, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image
        cv2.imshow('Color Detection', color_image)
        cv2.waitKey(1)

    def depth_callback(self, depth_msg):
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg)

        # Store the depth image
        self.depth_image = depth_image

    def get_depth(self, x, y):
        if self.depth_image is None:
            return 0.0

        # Get the depth value at the given (x, y) position
        depth = self.depth_image[y, x]

        return depth


def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()