#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.color_subscriber = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.color_callback, 10)
        self.depth_mask_publisher = self.create_publisher(Image, '/depth_mask', 10)

    def color_callback(self, color_msg):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        # Convert the color image to the HSV color space
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for pink color
        lower_pink = np.array([150, 50, 50], dtype=np.uint8)
        upper_pink = np.array([170, 255, 255], dtype=np.uint8)

        # Create a mask based on the pink color range
        pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

        # Apply the mask to the color image
        pink_pixels = cv2.bitwise_and(color_image, color_image, mask=pink_mask)

        # Convert the color image to grayscale
        gray_image = cv2.cvtColor(pink_pixels, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to create a binary mask
        _, binary_mask = cv2.threshold(gray_image, 1, 255, cv2.THRESH_BINARY)

        # Convert the binary mask to a ROS image message
        depth_mask_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')

        # Publish the depth mask message
        self.depth_mask_publisher.publish(depth_mask_msg)

        # Display the depth mask
        cv2.imshow('Depth Mask', binary_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()