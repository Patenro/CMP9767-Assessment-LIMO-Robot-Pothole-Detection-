#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt  # Import the matplotlib library
from rclpy.node import Node
from rclpy import qos
from cv2 import namedWindow, resize
from cv2 import COLOR_BGR2HSV, inRange
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

font = cv2.FONT_HERSHEY_SIMPLEX

class ImageConverter(Node):
    def __init__(self):
        super().__init__("Pothole_Severity_Detector")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.depth_image = None
        self.severity_counts = {'Slightly Severe': 0, 'Moderately Severe': 0, 'Highly Severe': 0, 'Dangerously Severe': 0}
        self.logger = self.get_logger()

        # Open log file for writing, overwriting previous content
        self.log_file_path = "severity_log.txt"
        with open(self.log_file_path, "w") as file:
            file.write("Pothole Severity Log\n")
        self.log_file = open(self.log_file_path, "a")

    def __del__(self):
        # Close the log file when the object is destroyed
        self.log_file.close()

    def get_severity_level(self, severity):
        if severity < 1500:
            return "Slightly Severe"
        elif 1500 <= severity <= 4000:
            return "Moderately Severe"
        elif 4001 <= severity <= 6000:
            return "Highly Severe"
        else:
            return "Dangerously Severe"

    def search_contours(self, mask):
        severities = []

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            severities.append(area)

            if area > 100:
                severity_level = self.get_severity_level(area)
                self.severity_counts[severity_level] += 1

                x, y, w, h = cv2.boundingRect(contour)
                centroid_x = x + (w // 2)
                centroid_y = y + (h // 2)

                pixel_color = tuple(self.cv_image[centroid_y, centroid_x])

                # Append severity level to the log file
                self.log_file.write(f"Pothole Severity: {area:.2f}, Level: {severity_level}\n")
                self.log_file.flush()

        return severities

    def plot_severity_graph(self):
        severity_levels = list(self.severity_counts.keys())
        count_values = list(self.severity_counts.values())

        # Plotting the bar graph
        plt.bar(severity_levels, count_values, color='blue')
        plt.xlabel('Severity Level')
        plt.ylabel('Count')
        plt.title('Count of Potholes by Severity Level')

        # Save the plot to an image file
        plt.savefig('severity_plot.png')
        plt.close()

    def image_callback(self, data):
        namedWindow("Image window showing the distance to each pothole and their severity")

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        severities = self.search_contours(mask)

        cv2.imshow("Image window showing the distance to each pothole and their severity", self.cv_image)
        cv2.waitKey(1)

        # Plot the severity graph
        self.plot_severity_graph()

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
