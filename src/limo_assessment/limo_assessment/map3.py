#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy import qos
from cv2 import namedWindow, resize, INTER_CUBIC
from cv2 import imshow, waitKey, cvtColor, COLOR_BGR2HSV, inRange
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
import plotly.graph_objects as go
from plotly.subplots import make_subplots
font = cv2.FONT_HERSHEY_SIMPLEX


class PotholeMappingNode(Node):
    def __init__(self):
        super().__init__('pothole_mapping_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )  # Set QoS Profile

        self.odom_sub = self.create_subscription(
            PoseStamped,
            "/odom",
            self.odom_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )  # Set QoS Profile

        self.depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )  # Set QoS Profile

        self.depth_image = None
        self.potholes = []
        self.robot_initial_pose = None

        # Set up Plotly figure for visualization
        self.fig = make_subplots(rows=1, cols=2, subplot_titles=('Pothole Map', 'Pothole Distances'))
        self.fig.update_layout(height=600, width=1200)

        # Publisher for the occupancy grid map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            10
        )

        # Transform broadcaster for robot's initial pose
        self.tf_broadcaster = TransformBroadcaster(self)

    def image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, COLOR_BGR2HSV)

        lower_pothole = np.array([150, 50, 50])
        upper_pothole = np.array([170, 255, 255])

        mask = inRange(hsv, lower_pothole, upper_pothole)

        # Search for contours and update pothole positions
        self.search_contours(mask)

        # Update and publish occupancy grid map
        self.update_occupancy_grid()

        # Plot the potholes on the map
        self.plot_potholes()

        # Display the Plotly figure
        self.fig.show()

    def odom_callback(self, data):
        # Store the robot's initial pose
        if self.robot_initial_pose is None:
            self.robot_initial_pose = data.pose

        # Broadcast the transform for the robot's initial pose
        transform_stamped = self.create_transform_stamped(data.header, self.robot_initial_pose)
        self.tf_broadcaster.sendTransform(transform_stamped)

    def create_transform_stamped(self, header, pose):
        transform_stamped = PoseStamped()
        transform_stamped.header = Header()
        transform_stamped.header.stamp = header.stamp
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.pose = pose
        return transform_stamped

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    def search_contours(self, mask):
        contours_count = 0
        contours_area = []

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            contours_count += 1

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Add pothole position to the list
            self.potholes.append((cX, cY))

            # Draw contour on the original image
            cv2.drawContours(self.cv_image, [contour], -1, (0, 255, 0), 2)

            # Add point at the center of the contour
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)

            if self.depth_image is not None:
                depth_at_center = self.depth_image[cY, cX]
                distance_mm = depth_at_center * 1000  # Convert to millimeters
                cv2.putText(
                    self.cv_image,
                    f"{distance_mm:.2f}mm",
                    (cX + 10, cY + 10),
                    font,
                    0.5,
                    (255, 255, 255),
                    2,
                )

    def update_occupancy_grid(self):
        # Update occupancy grid based on pothole positions
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = 0.1  # Adjust as needed
        occupancy_grid.info.width = 100  # Adjust as needed
        occupancy_grid.info.height = 100  # Adjust as needed
        occupancy_grid.info.origin.position.x = -occupancy_grid.info.width / 2.0 * occupancy_grid.info.resolution
        occupancy_grid.info.origin.position.y = -occupancy_grid.info.height / 2.0 * occupancy_grid.info.resolution
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Initialize the occupancy grid data with zeros
        occupancy_map = np.zeros((occupancy_grid.info.height, occupancy_grid.info.width), dtype=np.int8)

        # Iterate over each pothole position and update the occupancy grid
        for pothole_x, pothole_y in self.potholes:
            # Convert pothole position to grid coordinates
            grid_x = int((pothole_x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution)
            grid_y = int((pothole_y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution)

            # Ensure that the grid coordinates are within the grid boundaries
            if 0 <= grid_x < occupancy_grid.info.width and 0 <= grid_y < occupancy_grid.info.height:
                # Increment the occupancy value at the corresponding grid cell
                occupancy_map[grid_y, grid_x] += 1

        # Flatten the occupancy map and assign it to the occupancy grid data
        occupancy_grid.data = occupancy_map.flatten().tolist()

        # Publish the occupancy grid map
        self.map_publisher.publish(occupancy_grid)

    def plot_potholes(self):
        # Plot pothole positions
        robot_x = self.robot_initial_pose.position.x if self.robot_initial_pose else 0.0
        robot_y = self.robot_initial_pose.position.y if self.robot_initial_pose else 0.0

        for i, (pothole_x, pothole_y) in enumerate(self.potholes):
            distance_to_robot = np.sqrt((pothole_x - robot_x) ** 2 + (pothole_y - robot_y) ** 2)
            color = f'rgba({i * 10 % 255}, {i * 20 % 255}, {i * 30 % 255}, 1)'

            # Plot pothole location with a unique color based on distance
            self.fig.add_trace(go.Scatter(x=[pothole_x], y=[pothole_y], mode='markers',
                                         marker=dict(size=10, color=color),
                                         name=f'Pothole {i + 1}'))

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    pothole_mapping_node = PotholeMappingNode()
    pothole_mapping_node.run()
    pothole_mapping_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
