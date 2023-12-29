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
        self.color_counts = {color: 0 for color in self.colors.keys()}

        # Subscribe to the camera image topic
        self.image_subscription = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            10
        )

        # Publisher for the occupancy grid map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map_new",
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
            contours_count = 0
            contours_area = []
            for contour in contours:
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

                area = cv2.contourArea(contour)
                contours_area.append(area)

                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                cv2.putText(
                    frame,
                    f"{contours_count}",
                    (cX - 25, cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

                contours_count += 1

        # Publish the occupancy grid map
        occupancy_grid = self.create_occupancy_grid(frame)
        self.map_publisher.publish(occupancy_grid)

        # Display the frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        # Print the sum total
        print("Sum Total:", sum_total)

    def create_occupancy_grid(self, frame):
        # Get the dimensions of the frame
        height, width, _ = frame.shape

        # Create an empty occupancy grid map
        occupancy_map = np.zeros((height, width), dtype=np.int8)

        # Iterate over each pixel in the frame
        for y in range(height):
            for x in range(width):
                pixel = frame[y, x]
                for color, rgb in self.colors.items():
                    if np.array_equal(pixel, rgb):
                        occupancy_map[y, x] = 100  # Set cell as occupied
                        self.color_counts[color] += 1  # Increment color count
                        break
                else:
                    occupancy_map[y, x] = -1  # Set cell as unknown

        # Create an OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = 0.05  # Adjust as needed
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Convert the occupancy map to a 1D array
        occupancy_data = occupancy_map.flatten().tolist()
        occupancy_grid.data = occupancy_data

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