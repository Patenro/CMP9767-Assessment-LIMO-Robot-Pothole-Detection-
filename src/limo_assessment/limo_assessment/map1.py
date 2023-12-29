#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy import qos
from cv2 import resize, INTER_CUBIC
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

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
            Odometry,
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
        self.robot_current_pose = None

        # Initialize Matplotlib figure and axis
        self.fig, self.ax = plt.subplots()

    def image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = resize(self.cv_image, None, fx=1, fy=1, interpolation=INTER_CUBIC)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_pothole = np.array([150, 50, 50])
        upper_pothole = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_pothole, upper_pothole)

        # Search for contours and update pothole positions
        self.search_contours(mask)

        # Plot the potholes on the map
        self.plot_potholes()

    def odom_callback(self, data):
        # Store the robot's initial pose
        if self.robot_initial_pose is None:
            self.robot_initial_pose = data.pose.pose

        # Store the robot's current pose
        self.robot_current_pose = data.pose.pose

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    def search_contours(self, mask):
        contours_count = 0

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
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

    def plot_potholes(self):
        # Plot pothole positions
        if self.robot_current_pose is not None:
            robot_x = self.robot_current_pose.position.x * 1000  # Convert to millimeters
            robot_y = self.robot_current_pose.position.y * 1000  # Convert to millimeters

            for i, (pothole_x, pothole_y) in enumerate(self.potholes):
                # Calculate pothole position relative to the robot
                pothole_rel_x = pothole_x - self.robot_current_pose.position.x * 1000  # Convert to millimeters
                pothole_rel_y = pothole_y - self.robot_current_pose.position.y * 1000  # Convert to millimeters

                # Plot pothole location on the map
                self.ax.scatter(
                    [robot_x + pothole_rel_x], [robot_y + pothole_rel_y],
                    c='b', marker='o', label=f'Pothole {i + 1}'
                )

            # Plot the robot's current position
            self.ax.scatter([robot_x], [robot_y], c='r', marker='^', label='Robot')

            # Set axis limits and labels
            self.ax.set_xlabel('X (millimeters)')
            self.ax.set_ylabel('Y (millimeters)')
            self.ax.legend()

            # Show the Matplotlib plot
            plt.pause(0.01)
            plt.draw()

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
