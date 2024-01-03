#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy import qos
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2HSV, blur, Canny, resize, INTER_CUBIC
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

font = cv2.FONT_HERSHEY_SIMPLEX

class ImageConverter(Node):
    def __init__(self):
        super().__init__("opencv_test")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )  # Set QoS Profile

        self.depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )  # Set QoS Profile

        self.depth_image = None
        self.unique_colors = {}

    def search_contours(self, mask):
        contours_count = 0
        contours_area = []

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        closest_contour = None
        closest_distance = float("inf")  # Initialize with a large value

        for contour in contours:
            cv2.drawContours(self.cv_image, [contour], -1, (0, 255, 0), 2)
            contours_count += 1

            area = cv2.contourArea(contour)
            contours_area.append(area)

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.putText(
                self.cv_image,
                f"{contours_count}",
                (cX - 25, cY - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            # Add point at the center of the contour
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)

            if self.depth_image is not None:
                depth_at_center = self.depth_image[cY, cX]
                distance_mm = depth_at_center * 1000  # Convert to millimeters
                cv2.putText(
                    self.cv_image,
                    f"{distance_mm:.2f}mm",
                    (cX + 10, cY + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

                # Update closest contour based on distance
                if distance_mm < closest_distance:
                    closest_distance = distance_mm
                    closest_contour = contour

            # Count and track unique colors
            if area > 100:  # Adjust the area threshold as needed
                x, y, w, h = cv2.boundingRect(contour)
                centroid_x = x + (w // 2)
                centroid_y = y + (h // 2)
                pixel_color = tuple(self.cv_image[centroid_y, centroid_x])
                if pixel_color not in self.unique_colors:
                    self.unique_colors[pixel_color] = 1

        unique_colors_count = len(self.unique_colors)
        print("Number of unique colors:", unique_colors_count)

        return contours_count, contours_area, closest_contour

    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("Masked")
        #namedWindow("Distance")

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(
            self.cv_image, None, fx=1, fy=1, interpolation=INTER_CUBIC
        )

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        count, areas, closest_contour = self.search_contours(mask)

        cv2.imshow("Image window", self.cv_image)
        cv2.imshow("Masked", mask)
        cv2.waitKey(1)

        print("Number of contours:", count)
        print("Areas of contours:", areas)

        if closest_contour is not None:
            severity = cv2.contourArea(closest_contour)
            print("Severity of closest color:", severity)
        
    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)


def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()