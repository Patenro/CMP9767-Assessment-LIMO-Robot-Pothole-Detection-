#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover(Node):
    def __init__(self):#Initialize the node
        super().__init__('navigation')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10) #Publish to /cmd_vel
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)#Subscribe to /scan
        self.get_logger().info('THE ROBOT IS AUTONOMOUS NOW')#Log that the robot is autonomous
    
    def laserscan_callback(self, data):
        min_dist = min(data.ranges[int(len(data.ranges)/2) -30 : int(len(data.ranges)/2) +30])#Get the minimum distance
        left = data.ranges[int(len(data.ranges)/2) -60] #Get the left distance
        right = data.ranges[int(len(data.ranges)/2) +60]#Get the right distance

        # print("Min: ", min_dist)
        t = Twist()
        # if min_dist < 0.5 and distance of the objects on the left is greater than the right, turn left
        if min_dist < 0.45 and left > right:
            t.angular.z = -0.5
        # if min_dist < 0.5 and distance of the objects on the right is greater than the left, turn right
        elif min_dist < 0.45 and right > left:
            t.angular.z = 0.5
        # if min_dist < 0.5 and distance of the objects on the left and right are equal, go backwards
        elif min_dist < 0.45 and right == left:
            t.linear.x = -0.2
        # if min_dist < 0.3, go backwards
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
