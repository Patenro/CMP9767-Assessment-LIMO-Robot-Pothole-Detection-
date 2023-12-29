#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from cv2 import namedWindow, cvtColor, imshow, inRange, destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2HSV, blur, Canny, resize, INTER_CUBIC
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        self.bridge = CvBridge()
        self.depth_image = None
        self.unique_colors = {}

        self.ct = cv2.TrackerCSRT_create()
        self.colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
        self.color_counts = {'red': 0, 'green': 0, 'blue': 0}

        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.depth_callback,
            10
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            10
        )

        namedWindow("Image window")
        namedWindow("Masked")

    def search_contours(self, mask):
        contours_count = 0
        contours_area = []

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

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

            cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)

            if self.depth_image is not None:
                depth_at_center = self.depth_image[cY, cX]
                distance_mm = depth_at_center * 1000
                cv2.putText(
                    self.cv_image,
                    f"{distance_mm:.2f}mm",
                    (cX + 10, cY + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                centroid_x = x + (w // 2)
                centroid_y = y + (h // 2)
                pixel_color = tuple(self.cv_image[centroid_y, centroid_x])
                if pixel_color not in self.unique_colors:
                    self.unique_colors[pixel_color] = 1

        unique_colors_count = len(self.unique_colors)
        print("Number of unique colors:", unique_colors_count)

        return contours_count, contours_area

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cv_image = resize(
            self.cv_image, None, fx=1, fy=1, interpolation=INTER_CUBIC
        )

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        count, areas = self.search_contours(mask)

        cv2.imshow("Image window", self.cv_image)
        cv2.imshow("Masked", mask)
        cv2.waitKey(1)

        mean_area = mean(areas) if areas else 0
        print("Mean contour area:", mean_area)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def create_occupancy_grid(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = 0.05
        occupancy_grid.info.width = gray.shape[1]
        occupancy_grid.info.height = gray.shape[0]
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        data = []
        for row in gray:
            row_data = []
            for pixel in row:
                if pixel == 0:
                    row_data.append(-1)
                else:
                    row_data.append(100)
            data.extend(row_data)
        occupancy_grid.data = data

        return occupancy_grid

    def run(self):
        rclpy.spin(self)

        for color, count in self.color_counts.items():
            print(f"{color}: {count}")

        destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    combined_node = CombinedNode()
    combined_node.run()
    combined_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()