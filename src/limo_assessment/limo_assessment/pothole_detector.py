#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PotholeDetector(Node):
    def __init__(self):
        super().__init__('pothole_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply a Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection to find edges
        edges = cv2.Canny(blurred, 50, 150)

        # Perform contour detection
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Measure the potholes
        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Ignore small contours (noise)
            if area < 100:
                continue

            # Calculate the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Draw the bounding box on the original image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Print the area of the pothole
            self.get_logger().info(f'Pothole area: {area}')

        # Display the image with detected potholes
        cv2.imshow('Pothole Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    rclpy.spin(pothole_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()