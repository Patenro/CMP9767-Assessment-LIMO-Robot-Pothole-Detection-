#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Image,
            '/limo/depth_camera_link/depth/image_raw',
            self.image_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.turning = False

    def image_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Check if the image is grayscale (single channel)
        if len(cv_image.shape) == 2:
            # Convert grayscale image to three-channel (BGR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        # Check for obstacles
        obstacle_detected = self.detect_obstacle(cv_image)

        if obstacle_detected:
            if not self.turning:
                self.turn()
        else:
            self.move_forward()

    def detect_obstacle(self, cv_image):
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper pink color range
        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([170, 255, 255])

        # Threshold the image to obtain only pink pixels
        mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

        # Count the number of pink pixels
        num_pink_pixels = cv2.countNonZero(mask)

        # Set a threshold for obstacle detection (adjust as needed)
        obstacle_threshold = 100

        obstacle_detected = num_pink_pixels > obstacle_threshold
        return obstacle_detected

    def move_forward(self):
        # Move the robot forward
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Set the linear velocity for forward movement
        self.cmd_vel_pub.publish(twist_msg)
        self.turning = False

    def turn(self):
        # Turn the robot
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Set the angular velocity for turning
        self.cmd_vel_pub.publish(twist_msg)
        self.turning = True

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()