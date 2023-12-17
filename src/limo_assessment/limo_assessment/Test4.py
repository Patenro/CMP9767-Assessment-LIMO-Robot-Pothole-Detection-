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
        self.obstacle_on_left = False
        self.obstacle_on_right = False

    def lidar_callback(self, msg):
        avoidance_cmd = Twist()
        obstacle_distance = min(msg.ranges)

        self.obstacle_on_left = False
        self.obstacle_on_right = False

        if obstacle_distance < 0.03:  # 3cm threshold
            # Check if there's an obstacle on the left
            if any(obstacle_distance < 0.03 for obstacle_distance in msg.ranges[0:180]):
                self.obstacle_on_left = True
            # Check if there's an obstacle on the right
            if any(obstacle_distance < 0.03 for obstacle_distance in msg.ranges[180:360]):
                self.obstacle_on_right = True

        if self.obstacle_on_left:
            avoidance_cmd.linear.x = 0.0  # Stop moving forward
            avoidance_cmd.angular.z = -0.5  # Turn left to avoid obstacle
        elif self.obstacle_on_right:
            avoidance_cmd.linear.x = 0.0  # Stop moving forward
            avoidance_cmd.angular.z = 0.5  # Turn right to avoid obstacle
        else:
            avoidance_cmd.linear.x = 0.2  # Move forward
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