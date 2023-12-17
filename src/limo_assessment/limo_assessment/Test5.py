#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace '/scan' with the actual LIDAR topic name
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_in_front = False

    def lidar_callback(self, msg):
        avoidance_cmd = Twist()
        min_range = min(msg.ranges)

        if min_range < msg.range_min or min_range > msg.range_max:
            min_range = msg.range_max + 1.0  # Set to a value greater than range_max to ignore

        if min_range < 0.3:  # Adjust the distance threshold as needed
            self.obstacle_in_front = True
        else:
            self.obstacle_in_front = False

        if self.obstacle_in_front:
            avoidance_cmd.linear.x = 0.0  # Stop moving forward
            avoidance_cmd.angular.z = 0.5  # Rotate to avoid obstacle
        else:
            avoidance_cmd.linear.x = 0.3  # Move forward
            avoidance_cmd.angular.z = 0.0  # No rotation

        self.publisher.publish(avoidance_cmd)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()