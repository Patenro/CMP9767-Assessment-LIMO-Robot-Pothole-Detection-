#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PoseMarkerSubscriber(Node):

    def __init__(self):
        super().__init__('pose_marker_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/limo/p_color_pose',
            self.pose_callback,
            10)

        self.marker_publisher = self.create_publisher(Marker, '/limo/pose_marker', 10)
        self.marker_id_counter = 0  # counter for assigning unique IDs to markers

    def pose_callback(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 0.1  # adjust the marker size as needed
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red color

        # Assign a unique ID to the marker for each detected pose
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    pose_marker_subscriber = PoseMarkerSubscriber()
    rclpy.spin(pose_marker_subscriber)
    pose_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
