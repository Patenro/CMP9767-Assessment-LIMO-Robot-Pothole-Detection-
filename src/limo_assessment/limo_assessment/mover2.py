#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover(Node):
    """
    A very simple Roamer implementation for LIMO.
    It simply goes straight until any obstacle is within
    2 m distance and then just simply turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('autonomous_navigation')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
        self.get_logger().info('THE ROBOT IS AUTONOMOUS NOW')
    
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        min_dist = min(data.ranges[int(len(data.ranges)/2) -30 : int(len(data.ranges)/2) +30])
        left = data.ranges[int(len(data.ranges)/2) -60]
        right = data.ranges[int(len(data.ranges)/2) +60]

        min_dist_left = left
        min_dist_right = right
        
        # print("Min: ", min_dist)
        t = Twist()
        if min_dist < 0.45 and min_dist_left > min_dist_right:
            t.angular.z = -0.5
        elif min_dist < 0.45 and min_dist_right > min_dist_left:
            t.angular.z = 0.5
        elif min_dist < 0.45 and min_dist_right == min_dist_left:
            t.linear.x = -0.2
        elif min_dist < 0.3:
            t.linear.x = -0.5
        else:
            t.linear.x = 0.2
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
