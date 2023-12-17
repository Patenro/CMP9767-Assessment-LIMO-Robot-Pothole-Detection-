#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')

        # Initialize the centroid tracker and other variables
        self.ct = cv2.TrackerCSRT_create()
        self.colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
        self.color_counts = {'red': 0, 'green': 0, 'blue': 0}

        # Subscribe to the camera image topic
        self.image_subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )
        self.image_subscription  # Prevent unused variable warning

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS Image message to a BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        sum_total = 0
        identified_color = set()

        # Detect colors in the frame
        for color, rgb in self.colors.items():
            lower = np.array([150, 50, 50], dtype=np.uint8)
            upper = np.array([170, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Update color counts using contour areas
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Adjust the area threshold as needed
                    self.color_counts[color] += 1
                    sum_total += 1
                    identified_color.add(color)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), rgb, 2)

        # Display the frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)
        print("Number of contours:", sum_total)

    def run(self):
        rclpy.spin(self)

        # Print the color counts
        for color, count in self.color_counts.items():
            print(f"{color}: {count}")

def main(args=None):
    rclpy.init(args=args)
    color_tracker_node = ColorTrackerNode()
    color_tracker_node.run()
    color_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()