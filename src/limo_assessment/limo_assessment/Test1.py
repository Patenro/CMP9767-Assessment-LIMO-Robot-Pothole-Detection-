#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class PoseMarkerSubscriber(Node):

    def __init__(self):
        super().__init__('pose_marker_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/limo/p_color_pose',
            self.pose_callback,
            10)

        self.marker_publisher = self.create_publisher(MarkerArray, '/limo/pose_markers', 10)
        self.marker_id_counter = 0  # counter for assigning unique IDs to markers
        self.active_markers = {}

        # Subscribe to the node shutdown event
        self.get_logger().info("Subscribing to shutdown event")
        self.get_logger().info("To remove markers on shutdown")

        self.create_timer(1.0, self.publish_markers_timer)

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

        self.active_markers[marker.id] = marker
        self.publish_markers()

    def publish_markers_timer(self):
        # Periodically publish markers to keep them active
        self.publish_markers()

    def publish_markers(self):
        markers_msg = MarkerArray()
        for marker_id, marker in self.active_markers.items():
            markers_msg.markers.append(marker)

        self.marker_publisher.publish(markers_msg)

    def on_shutdown(self):
        # Remove markers on node shutdown
        self.get_logger().info("Removing markers on shutdown")
        markers_msg = MarkerArray()
        for marker_id, marker in self.active_markers.items():
            # Set action to DELETE for each marker
            marker.action = Marker.DELETE
            markers_msg.markers.append(marker)

        self.marker_publisher.publish(markers_msg)

def main(args=None):
    rclpy.init(args=args)
    pose_marker_subscriber = PoseMarkerSubscriber()
    rclpy.spin(pose_marker_subscriber)
    pose_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
