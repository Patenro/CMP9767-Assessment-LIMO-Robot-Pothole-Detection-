#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt
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
        self.unique_colors = {}
        self.logger = self.get_logger()
        self.logger.info('REPORT NODE INITIALIZED')

        # Open log file for writing, overwriting previous content
        self.log_file_path = "severity_log.txt"
        with open(self.log_file_path, "w") as file:
            file.write("Pothole Severity Log\n")
        self.log_file = open(self.log_file_path, "a")

        self.severities_list = []  # Initialize list to store severities

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
        contours_area = []
        severities = []

        colormap = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(1, -1), cv2.COLORMAP_JET)
        colormap = colormap.squeeze()

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        closest_contour = None
        closest_distance = float("inf")

        for contour in contours:
            cv2.drawContours(self.cv_image, [contour], -1, (0, 255, 0), 2)

            area = cv2.contourArea(contour)
            contours_area.append(area)
            
            # Calculate the center of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)

            if self.depth_image is not None:
                depth_at_center = self.depth_image[cY, cX]
                distance_mm = depth_at_center * 1000

                cv2.putText(
                    self.cv_image,
                    f"{distance_mm:.2f}mm",
                    (cX + 10, cY + 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

                min_distance = 300
                max_distance = 450

                if min_distance <= distance_mm <= max_distance and distance_mm < closest_distance:
                    closest_distance = distance_mm
                    closest_contour = contour

                severities.append(area)

                max_severity = max(severities)
                normalized_severity = int(area * 255 / max_severity) if max_severity != 0 else 0

                color = tuple(map(int, colormap[normalized_severity]))

                cv2.putText(
                    self.cv_image,
                    f"Severity: {area:.2f}",
                    (cX + 10, cY + 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

                if area > 100:
                    x, y, w, h = cv2.boundingRect(contour)
                    centroid_x = x + (w // 2)
                    centroid_y = y + (h // 2)
                    pixel_color = tuple(self.cv_image[centroid_y, centroid_x])
                    if pixel_color not in self.unique_colors:
                        self.unique_colors[pixel_color] = 1

        return contours, contours_area, severities, closest_contour

    def image_callback(self, data):
        namedWindow("Image window showing the distance to each pothole and their severity")

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        contours, contours_area, severities, closest_contour = self.search_contours(mask)

        cv2.imshow("Image window showing the distance to each pothole and their severity", self.cv_image)
        cv2.waitKey(1)

        if closest_contour is not None:
            severity = cv2.contourArea(closest_contour)
            severity_level = self.get_severity_level(severity)
            #self.logger.info(f"Severity of closest color: {severity}, Level: {severity_level}")
            self.log_file.write(f"Pothole Severity: {severity}, Level: {severity_level}\n")
            self.log_file.flush()

            # Only add severity of the closest contour to the list
            self.severities_list.append(severity)

            # Plot the bar graph for all contours
            self.plot_bar_graph()

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

    def plot_bar_graph(self):
        if self.severities_list:
            # Count occurrences of each severity level in the list
            severity_levels = ["Slightly Severe", "Moderately Severe", "Highly Severe", "Dangerously Severe"]
            severity_values = [self.get_severity_level(severity) for severity in self.severities_list]
            severity_counts = {level: severity_values.count(level) for level in severity_levels}

            # Plot the bar graph
            plt.bar(severity_counts.keys(), severity_counts.values(), color=['blue', 'orange', 'green', 'red'])
            plt.xlabel('Severity Level')
            plt.ylabel('Count')
            plt.title('Pothole Severity Distribution Report')
            plt.savefig('pothole_severity_report.png')
            plt.show(block=False)
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()

    try:
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        pass

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
