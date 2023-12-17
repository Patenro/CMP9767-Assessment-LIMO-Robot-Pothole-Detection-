#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

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

        # Publisher for the occupancy grid map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            10
        )

        self.bridge = CvBridge()

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

    def create_occupancy_grid(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Create an OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = 0.05  # Adjust as needed
        occupancy_grid.info.width = gray.shape[1]
        occupancy_grid.info.height = gray.shape[0]
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
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

    def run(self):
        rclpy.spin(self)

        # Print the color counts
        for color, count in self.color_counts.items():
            print(f"{color}: {count}")

def main(args=None):
    rclpy.init(args=args)
    color_mapping_node = ColorMappingNode()
    color_mapping_node.run()
    color_mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
