#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        #Creates a timer that calls timer_callback() every 1.0 seconds
        self.create_timer(1.0, self.timer_callback)
        #To actually make the node work
        #self.get_logger().info('Welcome to ROS2')
    #to create a timer to call a specific function
    def timer_callback(self):
        self.get_logger().info('Hello ROS2')
def main(args=None):
    rclpy.init(args=args)
    #node
    node = MyNode()
    #To keep the node alive and running forever until you kill it with ctrl+c
    rclpy.spin(node)
    #lastline
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()