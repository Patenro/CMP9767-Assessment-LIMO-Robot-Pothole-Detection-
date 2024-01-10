#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math

class PoseMarkerSubscriber(Node):

    def __init__(self):
        super().__init__('marker')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/limo/p_color_pose',
            self.pose_callback,
            10)

        self.marker_publisher = self.create_publisher(Marker, '/limo/pose_marker', 10)
        self.marker_id_counter = 1  # counter for assigning unique IDs to markers
        self.marker_list = []  # to store the existing markers
        self.get_logger().info("POTHOLE LOCATIONS ARE BEING DETECTED AND MARKED ON THE MAP")

    def calculate_distance(self, pose1, pose2):
        # Calculate Euclidean distance between two poses
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def pose_callback(self, msg):
        # Check if the new pose is far enough from existing poses
        current_pose = msg.pose
        is_far_enough = all(
            self.calculate_distance(current_pose, marker.pose) > 0.27
            for marker in self.marker_list
        )

        if is_far_enough:
            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = current_pose
            marker.pose.position.z = 0.0  # Set z coordinate to 0
            marker.scale.x = 0.05  # adjust the marker size as needed
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red color

            # Assign a unique ID to the marker for each detected pose
            marker.id = self.marker_id_counter
            self.marker_id_counter += 1

            # Add the new marker to the list
            self.marker_list.append(marker)

            self.marker_publisher.publish(marker)

            # Create a txt file with coordinates and labels
            with open('pothole_coordinates.txt', 'w') as file:
                file.write('Pothole Coordinates\n')
                for marker in self.marker_list:
                    file.write(f'Pothole {marker.id}: {marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z}\n')


def main(args=None):
    rclpy.init(args=args)
    pose_marker_subscriber = PoseMarkerSubscriber()
    rclpy.spin(pose_marker_subscriber)
    pose_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
