#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import yaml
import os

class ColorMappingNode(Node):
    def __init__(self):
        super().__init__('color_mapping_node')

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

        # Subscribe to the LIDAR scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/limo/lidar",
            self.scan_callback,
            10
        )
        self.scan_subscription  # Prevent unused variable warning

        # Publisher for the occupancy grid map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            10
        )

        self.bridge = CvBridge()

        # Map variables
        self.map_resolution = 0.05  # Adjust as needed
        self.map_size_x = 1000  # Adjust as needed
        self.map_size_y = 1000  # Adjust as needed
        self.map = np.zeros((self.map_size_x, self.map_size_y), dtype=np.uint8)

    def image_callback(self, msg):
        # Convert the ROS Image message to a BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Initialize the sum total
        sum_total = 0

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
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), rgb, 2)

        # Publish the occupancy grid map
        occupancy_grid = self.create_occupancy_grid(frame)
        self.map_publisher.publish(occupancy_grid)

        # Display the frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        # Print the sum total
        print("Sum Total:", sum_total)

    def scan_callback(self, msg):
        # Calculate the map indices for each LIDAR point
        x_values = np.array([msg.ranges[i] * np.cos(msg.angle_min + i * msg.angle_increment)
                            for i in range(len(msg.ranges))])
        y_values = np.array([msg.ranges[i] * np.sin(msg.angle_min + i * msg.angle_increment)
                            for i in range(len(msg.ranges))])
        indices_x = np.round((self.map_size_x / 2) - (x_values / self.map_resolution)).astype(int)
        indices_y = np.round((self.map_size_y / 2) + (y_values / self.map_resolution)).astype(int)

        # Update the map with occupied points
        self.map[indices_x, indices_y] = 255

    def create_occupancy_grid(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Create an OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_size_x
        occupancy_grid.info.height = self.map_size_y
        occupancy_grid.info.origin.position.x = -self.map_size_x * self.map_resolution / 2
        occupancy_grid.info.origin.position.y = -self.map_size_y * self.map_resolution / 2
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Convert the grayscale image to occupancy grid data
        data = []
        for row in gray:
            row_data = []
            for pixel in row:
                if pixel == 0:
                    row_data.append(-1)  # Unknown
                else:
                    row_data.append(100)  # Occupied
            data.extend(row_data)
        occupancy_grid.data = data

        return occupancy_grid



    def save_map(self):
        # Create a map visualization
        map_visualization = np.zeros((self.map_size_x, self.map_size_y), dtype=np.uint8)
        map_visualization[self.map == 0] = 255  # Free space (white)
        map_visualization[self.map == 255] = 0  # Occupied space (black)

        # Save the map as a PGM file
        map_filename = os.path.join(os.path.dirname(__file__), "map.pgm")
        with open(map_filename, "wb") as pgm_file:
            pgm_file.write(b'P5\n')
            pgm_file.write(f'{self.map_size_x} {self.map_size_y}\n'.encode())
            pgm_file.write(b'255\n')
            pgm_file.write(map_visualization.tobytes())

        # Save the map as a YAML file
        yaml_data = {
            'image': 'map.pgm',
            'resolution': self.map_resolution,
            'origin': [-self.map_size_x * self.map_resolution / 2, -self.map_size_y * self.map_resolution / 2, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        yaml_filename = os.path.join(os.path.dirname(__file__), "map.yaml")
        with open(yaml_filename, "w") as yaml_file:
            yaml.dump(yaml_data, yaml_file)

    def run(self):
        rclpy.spin(self)

        # Find the color with the maximum count
        max_count_color = max(self.color_counts, key=self.color_counts.get)
        max_count = self.color_counts[max_count_color]
        print(f"The color with the maximum count is {max_count_color} with a count of {max_count}.")

        # Save the map
        self.save_map()

def main(args=None):
    rclpy.init(args=args)
    color_mapping_node = ColorMappingNode()
    color_mapping_node.run()
    color_mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()