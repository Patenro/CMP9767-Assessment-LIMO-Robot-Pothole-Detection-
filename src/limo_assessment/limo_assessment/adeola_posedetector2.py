#!/usr/bin/env python3
# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2
import numpy as np

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ratio between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = 1.0  # for a simulated camera

    def __init__(self):
        super().__init__('image_projection_3')
        self.bridge = CvBridge()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                        self.camera_info_callback,
                                                        qos_profile=qos.qos_profile_sensor_data)

        self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)
        self.detected_color_pose_pub = self.create_publisher(PoseStamped, '/limo/detected_color_pose', 10)
        self.p_color_pose_pub = self.create_publisher(PoseStamped, '/limo/p_color_pose', 10)

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw',
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw',
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pink HSV color range
        self.lower_pink = np.array([150, 50, 50], dtype=np.uint8)
        self.upper_pink = np.array([170, 255, 255], dtype=np.uint8)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # convert images to OpenCV
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # convert color image to HSV
        image_hsv = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)

        # create a mask using the specified HSV color range for pink
        image_mask_pink = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)

        # find contours in the mask
        contours, _ = cv2.findContours(image_mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            print('No pink object detected.')
            return

        # find the largest contour
        max_contour = max(contours, key=cv2.contourArea)

        # calculate the bounding box around the contour
        x, y, w, h = cv2.boundingRect(max_contour)

        # calculate the centroid of the bounding box
        centroid_x = x + w // 2
        centroid_y = y + h // 2

        # check if centroid coordinates are within the image bounds
        if 0 <= centroid_x < image_color.shape[1] and 0 <= centroid_y < image_color.shape[0]:
            # "map" from color to depth image
            depth_coords = (
                image_depth.shape[0] / 2 + (centroid_y - image_color.shape[0] / 2) * self.color2depth_aspect,
                image_depth.shape[1] / 2 + (centroid_x - image_color.shape[1] / 2) * self.color2depth_aspect)

            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

            print('centroid coords: ', (centroid_x, centroid_y))
            print('depth coords: ', depth_coords)
            print('depth value: ', depth_value)

            # Check if the depth value is within the desired range (0.2 to 0.4 meters)
            if 0.2 <= depth_value <= 0.4:
                # calculate object's 3D location in camera coords
                camera_coords = self.camera_model.projectPixelTo3dRay(
                    (centroid_y, centroid_x))  # project the image coords (x,y) into 3D ray in camera coords
                camera_coords = [x / camera_coords[2] for x in camera_coords]  # adjust the resulting vector so that z = 1
                camera_coords = [x * depth_value for x in camera_coords]  # multiply the vector by depth

                print('camera coords: ', camera_coords)

                # define a point in camera coordinates
                object_location = PoseStamped()
                object_location.header.frame_id = "depth_link"
                object_location.pose.orientation.w = 1.0
                object_location.pose.position.x = camera_coords[0]
                # object_location.pose.position.y = camera_coords[1]
                object_location.pose.position.z = camera_coords[2]

                # publish the PoseStamped of the detected color
                self.detected_color_pose_pub.publish(object_location)

                # publish so we can see that in rviz
                self.object_location_pub.publish(object_location)

                # print out the coordinates in the odom frame
                transform = self.get_tf_transform('map', 'depth_link')
                if transform:
                    p_camera = do_transform_pose(object_location.pose, transform)

                    p_location = PoseStamped()
                    p_location.header.frame_id = "map"
                    p_location.pose = p_camera
                    p_location.pose.orientation.w = 1.0

                    self.p_color_pose_pub.publish(p_location)

                    print('odom coords: ', p_camera.position)

                if self.visualisation:
                    # draw bounding box
                    cv2.rectangle(image_color, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # resize and adjust for visualization
                    image_color = cv2.resize(image_color, (0, 0), fx=0.5, fy=0.5)
                    image_depth *= 1.0 / 10.0  # scale for visualization (max range 10.0 m)

                    cv2.imshow("image depth", image_depth)
                    cv2.imshow("image color", image_color)
                    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_projection = ObjectDetector()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
