#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ColorCountingNode(Node):
    def __init__(self):
        super().__init__("color_counting_node")
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Apply necessary pre-processing steps (e.g., resizing, noise reduction, enhancement)

        # Convert image to the desired color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for detection (example: red, green, blue)
        color_ranges = [
            ((150, 50, 50), (170, 255, 255))]

        # Count the number of color regions
        color_counts = [0] * len(color_ranges)

        for i, (lower, upper) in enumerate(color_ranges):
            mask = cv2.inRange(hsv_image, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color_counts[i] = len(contours)

        # Print the color counts
        for i, count in enumerate(color_counts):
            color_name = ["Potholes"][i]  # Replace with your color names
            self.get_logger().info(f"{color_name} count: {count}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorCountingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()