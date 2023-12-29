#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from ultralytics import YOLO

class ColorMappingNode(Node):
    def __init__(self):
        super().__init__('color_mapping_node')
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')  # Adjust the model file accordingly

        # Publisher for the combined occupancy grid map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map_combined",
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect colors using YOLOv8
        results = self.yolo_model(frame)

        # Process YOLOv8 results and create occupancy grid
        occupancy_grid = self.process_yolo_results(results, frame)

        # Publish the combined occupancy grid map
        self.map_publisher.publish(occupancy_grid)

    def process_yolo_results(self, results, frame):
        # Initialize an empty occupancy grid map
        height, width, _ = frame.shape
        occupancy_map = np.zeros((height, width), dtype=np.int8)

        # Dictionary to track unique colors
        unique_colors = {}

        for result in results.xyxy:
            class_id = int(result[-1])
            color_name = self.get_color_name(class_id)  # Implement this function
            if color_name is not None:
                # Update color counts
                unique_colors[color_name] = unique_colors.get(color_name, 0) + 1

                # Get bounding box coordinates
                x, y, w, h = result[:-1].cpu().numpy().astype(int)

                # Draw bounding box on the frame
                cv2.rectangle(frame, (x, y), (w, h), (0, 255, 0), 2)

                # Convert bounding box to occupancy grid coordinates
                x_center = (x + w) // 2
                y_center = (y + h) // 2

                # Set corresponding cell in the occupancy grid as occupied
                occupancy_map[y_center, x_center] = 100

        # Print color counts
        print("Color Counts:", unique_colors)

        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = 0.1  # Adjust the resolution as needed
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = occupancy_map.flatten().tolist()

        return occupancy_grid

    def get_color_name(self, class_id):
        # Implement this function to map YOLO class IDs to color names
        # You can use a predefined mapping or train YOLO with specific colors
        # For simplicity, let's assume you have a mapping dictionary named class_mapping
        class_mapping = {
            0: 'red',
            1: 'green',
            2: 'blue'
            # Add more color mappings as needed
        }
        return class_mapping.get(class_id)

def main(args=None):
    rclpy.init(args=args)
    color_mapping_node = ColorMappingNode()

    # Subscribe to the camera image topic
    color_mapping_node.create_subscription(
        Image,
        "/limo/depth_camera_link/image_raw",
        color_mapping_node.image_callback,
        10
    )

    rclpy.spin(color_mapping_node)
    color_mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
