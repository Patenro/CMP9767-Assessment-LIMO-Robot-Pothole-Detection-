#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image
from cv2 import namedWindow, imshow, waitKey, resize
import os

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

        # Open log file for writing, overwriting previous content
        self.log_file_path = "severity_log.txt"
        self.log_file = open(self.log_file_path, "w")
        self.log_file.write("Pothole Severity Log\n")


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

    def display_severity_level(self, image, area, centroid_x, centroid_y, distance_mm):
        severity_level = self.get_severity_level(area)

        # Only display text for colors close to the robot
        min_distance = 300
        max_distance = 450

        if min_distance <= distance_mm <= max_distance:
            text = f"Severity: {area:.2f}, Level: {severity_level}"

            font_scale = 0.5
            font_thickness = 2

            # Choose color based on severity level
            color = {
                "Slightly Severe": (0, 255, 0),    # Green
                "Moderately Severe": (0, 255, 255),  # Yellow
                "Highly Severe": (0, 0, 255),       # Red
                "Dangerously Severe": (128, 0, 128)  # Dark Purple
            }.get(severity_level, (255, 255, 255))  # Default to white for unknown levels

            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)

            text_x = int(max(0, centroid_x - text_width / 2))
            text_y = int(max(text_height, centroid_y - 30))

            cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, font_thickness)

            return color
        else:
            return None

    def colorize_mask(self, mask, severity_level):
        # Choose color for the mask based on severity level
        color = {
            "Slightly Severe": (0, 255, 0),    # Green
            "Moderately Severe": (0, 255, 255),  # Yellow
            "Highly Severe": (0, 0, 255),       # Red
            "Dangerously Severe": (128, 0, 128)  # Dark Purple
        }.get(severity_level, (255, 255, 255))  # Default to white for unknown levels

        color_mask = cv2.bitwise_and(np.ones_like(self.cv_image) * color, self.cv_image, mask=mask)
        return color_mask

    def search_contours(self, mask):
        contours_area = []
        severities = []

        colormap = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(1, -1), cv2.COLORMAP_JET)
        colormap = colormap.squeeze()

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        closest_contour = None
        closest_distance = float("inf")

        font_color = (0, 0, 0)  # Default color if no contour is found
        distance_mm = 0  # Initialize distance_mm here

        for contour in contours:
            cv2.drawContours(self.cv_image, [contour], -1, (0, 255, 0), 2)

            area = cv2.contourArea(contour)
            contours_area.append(area)

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

                # Display severity level without overlapping
                font_color = self.display_severity_level(self.cv_image, area, cX, cY + 30, distance_mm)

                if area > 100:
                    x, y, w, h = cv2.boundingRect(contour)
                    centroid_x = x + (w // 2)
                    centroid_y = y + (h // 2)
                    pixel_color = tuple(self.cv_image[centroid_y, centroid_x])
                    if pixel_color not in self.unique_colors:
                        self.unique_colors[pixel_color] = 1

                    # Append severity level to the log file
                    severity_level = self.get_severity_level(area)
                    self.log_file.write(f"Pothole Severity: {area:.2f}, Level: {severity_level}\n")
                    self.log_file.flush()

        return contours, contours_area, severities, closest_contour, font_color, distance_mm

    def image_callback(self, data):
        namedWindow("Image window showing the distance to each pothole and their severity")

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        contours, contours_area, severities, closest_contour, font_color, distance_mm = self.search_contours(mask)

        cv2.imshow("Image window showing the distance to each pothole and their severity", self.cv_image)
        cv2.imshow("Masked", mask)

        # Display the severity level in the mask image
        if closest_contour is not None:
            mask_image = self.colorize_mask(mask, self.get_severity_level(contours_area[0]))
            cv2.imshow("Severity Mask", mask_image)

        cv2.waitKey(1)

        if closest_contour is not None:
            severity = cv2.contourArea(closest_contour)
            self.logger.info(f"Severity of closest color: {severity}, Level: {self.get_severity_level(severity)}")
            self.log_file.write(f"Pothole Severity: {severity}, Level: {self.get_severity_level(severity)}\n")
            self.log_file.flush()

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
