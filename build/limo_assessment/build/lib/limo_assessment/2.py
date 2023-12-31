#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy import qos
from cv2 import namedWindow, resize
from cv2 import COLOR_BGR2HSV, inRange
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

font = cv2.FONT_HERSHEY_SIMPLEX


class ColorMappingNode(Node):
    def __init__(self):
        super().__init__('color_mapping_node')
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

        self.colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
        self.color_counts = {'red': 0, 'green': 0, 'blue': 0, 'custom_color': 0}

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map_new",
            10
        )

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

                    # Update color counts using contour areas
                    for color, rgb in self.colors.items():
                        if np.array_equal(pixel_color, rgb):
                            self.color_counts[color] += 1

        return contours, contours_area, severities, closest_contour

    def image_callback(self, data):
        namedWindow("Image window")
        # namedWindow("Masked")

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        contours, contours_area, severities, closest_contour = self.search_contours(mask)

        cv2.imshow("Image window", self.cv_image)
        # cv2.imshow("Masked", mask)
        cv2.waitKey(1)

        if closest_contour is not None:
            severity = cv2.contourArea(closest_contour)
            self.logger.info(f"Severity of closest color: {severity}")

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")


def main(args=None):
    rclpy.init(args=args)
    color_mapping_node = ColorMappingNode()
    rclpy.spin(color_mapping_node)
    color_mapping_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()