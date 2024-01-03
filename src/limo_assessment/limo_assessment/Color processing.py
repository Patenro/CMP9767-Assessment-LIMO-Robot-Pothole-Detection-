#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import StaticTransformBroadcaster, Buffer
from tf2_geometry_msgs import do_transform_pose
import image_geometry

class ColorMappingNode(Node):
    def __init__(self):
        super().__init__('color_mapping_node')

        self.bridge = CvBridge()
        self.camera_model = None
        self.image_depth_ros = None
        self.color_counter = 1
        self.odom_coords = None

        # Color range for detection
        self.lower_color = np.array([150, 50, 50], dtype=np.uint8)
        self.upper_color = np.array([170, 255, 255], dtype=np.uint8)

        # Subscribe to camera info, color image, and depth image topics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/camera_info',
            self.camera_info_callback,
            10
        )

        self.image_color_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.image_color_callback,
            10
        )

        self.image_depth_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/depth/image_raw',
            self.image_depth_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for color locations
        self.color_location_pub = self.create_publisher(
            PointStamped,
            '/limo/color_location',
            10
        )

        # TF Broadcaster for static transforms
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # Wait for camera_model and depth image to arrive
        if self.camera_model is None or self.image_depth_ros is None:
            return

        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print("Error converting images:", str(e))
            return

        # Process color blob
        result = self.process_color_blob(image_color, image_depth)
        if result:
            image_coords, depth_value = result

            # Calculate 3D coordinates
            camera_coords = self.calculate_3d_coordinates(image_coords, depth_value)

            # Broadcast TF frame on the detected color
            self.broadcast_tf_frame(f'color_frame_{self.color_counter}', 'depth_camera_link', image_coords, depth_value)

            # Publish color location
            self.publish_color_location(camera_coords)

    def process_color_blob(self, image_color, image_depth):
        # Detect a color blob in the color image
        image_mask = cv2.inRange(image_color, self.lower_color, self.upper_color)

        # Calculate moments of the binary image
        M = cv2.moments(image_mask)

        if M["m00"] == 0:
            print('No object detected.')
            return None

        # Calculate the y, x centroid
        image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
        # Get the depth reading at the centroid location
        depth_value = image_depth[int(image_coords[0]), int(image_coords[1])]  # You might need to do some boundary checking first!

        return image_coords, depth_value

    def calculate_3d_coordinates(self, image_coords, depth_value):
        # Calculate object's 3D location in camera coords
        camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))
        camera_coords = [x * depth_value for x in camera_coords]

        return camera_coords

    def broadcast_tf_frame(self, frame_id, parent_frame_id, image_coords, depth_value):
        # Convert coordinates to integers
        image_coords = (int(image_coords[0]), int(image_coords[1]))

        # Broadcast a static transform from color frame to depth camera frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame_id
        transform.child_frame_id = frame_id
        transform.transform.translation.x = float(image_coords[1])
        transform.transform.translation.y = float(image_coords[0])
        transform.transform.translation.z = float(depth_value)
        transform.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(transform)

    def publish_color_location(self, camera_coords):
        if camera_coords and self.odom_coords:
            # Calculate color location with respect to the /odom frame
            odom_relative_coords = (
                camera_coords[0] + self.odom_coords[0],
                camera_coords[1] + self.odom_coords[1],
                camera_coords[2] + self.odom_coords[2]
            )

            # Convert coordinates to integers
            odom_relative_coords = (
                int(odom_relative_coords[0]),
                int(odom_relative_coords[1]),
                int(odom_relative_coords[2])
            )

            # Publish color location with a unique identifier
            color_location = PointStamped()
            color_location.header.frame_id = "odom"
            color_location.point.x = float(odom_relative_coords[0])
            color_location.point.y = float(odom_relative_coords[1])
            color_location.point.z = float(odom_relative_coords[2])

            # Print the result
            self.get_logger().info(f'Color{self.color_counter}: x={odom_relative_coords[0]}, y={odom_relative_coords[1]}, z={odom_relative_coords[2]} with reference to the odom')

            self.color_location_pub.publish(color_location)

            # Increment color identifier counter
            self.color_counter += 1

    def odom_callback(self, msg):
        # Store the odometry coordinates
        self.odom_coords = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

def main(args=None):
    rclpy.init(args=args)
    color_mapping_node = ColorMappingNode()
    rclpy.spin(color_mapping_node)
    color_mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
