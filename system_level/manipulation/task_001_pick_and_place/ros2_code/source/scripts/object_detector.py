#!/usr/bin/env python3

"""
Detects objects based on their color and provides information
about their position, dimensions and color.

Migrated from ROS 1 to ROS 2 Humble.

Author: Elena Oikonomou (original ROS1)
Date:   Fall 2023
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, CameraInfo
from typing import List, Tuple

from pick_and_place.msg import DetectedObjectsStamped, DetectedObject

import threading
import time


def translation_matrix(direction):
    """Return matrix to translate by direction vector."""
    M = np.identity(4)
    M[:3, 3] = direction[:3]
    return M


def inverse_matrix(matrix):
    """Return inverse of square transformation matrix."""
    return np.linalg.inv(matrix)


class VisionObjectDetector(Node):
    def __init__(self):
        super().__init__('vision_object_detector')

        self.color_ranges = {
            "blue": [np.array([110, 50, 50]), np.array([130, 255, 255])],
            "green": [np.array([36, 25, 25]), np.array([70, 255, 255])],
            "red": [np.array([0, 100, 100]), np.array([10, 255, 255])],
            "black": [np.array([0, 0, 0]), np.array([180, 255, 40])]
        }
        self.block_contour_area_threshold = 200
        self.blocks_on_workbench = []

        # Declare parameters using ROS 2 style
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detection_topic', '/object_detection')
        self.declare_parameter('model_name', 'kinect')
        self.declare_parameter('contour_area_threshold', 200)

        self._image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self._camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self._detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self._model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.block_contour_area_threshold = self.get_parameter('contour_area_threshold').get_parameter_value().integer_value

        # Create service client for Gazebo model state (non-blocking)
        self._cb_group = ReentrantCallbackGroup()
        self._gazebo_available = False
        try:
            from gazebo_msgs.srv import GetModelState
            self._GetModelState = GetModelState
            self.model_state_client = self.create_client(
                GetModelState,
                '/gazebo/get_model_state',
                callback_group=self._cb_group
            )
            self._gazebo_available = True
        except ImportError:
            self.get_logger().warn('gazebo_msgs not available; Gazebo integration disabled.')
            self.model_state_client = None

        self._has_cv2 = False
        self._has_cv_bridge = False
        try:
            import cv2
            self._cv2 = cv2
            self._has_cv2 = True
        except ImportError:
            self.get_logger().warn('cv2 not available.')

        try:
            import cv_bridge
            self.bridge = cv_bridge.CvBridge()
            self._has_cv_bridge = True
        except ImportError:
            self.get_logger().warn('cv_bridge not available.')
            self.bridge = None

        self._color_image_data = None
        self._depth_image_data = None
        self._camera_info_data = None
        self._color_image_event = threading.Event()
        self._depth_image_event = threading.Event()
        self._camera_info_event = threading.Event()

        self._temp_color_sub = self.create_subscription(
            Image, self._image_topic, self._initial_color_cb, 10,
            callback_group=self._cb_group)
        self._temp_depth_sub = self.create_subscription(
            Image, self._depth_topic, self._initial_depth_cb, 10,
            callback_group=self._cb_group)
        self._temp_info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._initial_info_cb, 10,
            callback_group=self._cb_group)

        self._initialized = False
        self.T_c_w = None
        self.T_w_c = None
        self.image_height = 480
        self.image_width = 640
        self.pin_cam = None
        self.depth_image = None
        self.workbench_depth = None

        self.detected_objects_pub = self.create_publisher(
            DetectedObjectsStamped, '/object_detection', 10)

        self.image_sub = self.create_subscription(
            Image, self._image_topic, self.image_callback, 10,
            callback_group=self._cb_group)

        self.get_logger().info('VisionObjectDetector node created.')

    def _initial_color_cb(self, msg):
        if self._color_image_data is None:
            self._color_image_data = msg
            self._color_image_event.set()

    def _initial_depth_cb(self, msg):
        if self._depth_image_data is None:
            self._depth_image_data = msg
            self._depth_image_event.set()

    def _initial_info_cb(self, msg):
        if self._camera_info_data is None:
            self._camera_info_data = msg
            self._camera_info_event.set()

    def try_initialize(self):
        if self._initialized:
            return True
        if self._color_image_data is None or self._depth_image_data is None or self._camera_info_data is None:
            return False
        if not self._has_cv2 or not self._has_cv_bridge:
            return False
        try:
            self.T_c_w, self.T_w_c = self.get_camera_homogeneous_tranforms()
            if self.T_c_w is None:
                return False
            image = self.bridge.imgmsg_to_cv2(self._color_image_data, desired_encoding='bgr8')
            h, w, c = image.shape
            self.image_height = h
            self.image_width = w
            from image_geometry import PinholeCameraModel
            self.pin_cam = PinholeCameraModel()
            self.pin_cam.fromCameraInfo(self._camera_info_data)
            self.depth_image = self.bridge.imgmsg_to_cv2(self._depth_image_data, desired_encoding='32FC1')
            self.workbench_depth = self._compute_workbench_depth(image)
            self._initialized = True
            self.get_logger().info('VisionObjectDetector fully initialized.')
            return True
        except Exception as e:
            self.get_logger().warn(f'Initialization failed: {e}')
            return False

    def _compute_workbench_depth(self, image):
        cv2 = self._cv2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(hsv, "black")
        height, width = mask.shape
        for i in range(height):
            for j in range(width):
                if mask[i, j] == 255:
                    return self.depth_image[int(i), int(j)]
        return 1.0

    def get_image_dimensions(self) -> Tuple[int]:
        return self.image_height, self.image_width

    def get_camera_homogeneous_tranforms(self) -> np.ndarray:
        camera_origin = self.get_model_position_from_gazebo(self._model_name)
        if camera_origin is None:
            return None, None
        Rot_c_w = np.array([[0, -1, 0, 0],
                            [-1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])
        Transl_c_w = translation_matrix(camera_origin)
        T_c_w = np.dot(Transl_c_w, Rot_c_w)
        T_w_c = inverse_matrix(T_c_w)
        return T_c_w, T_w_c

    def get_model_position_from_gazebo(self, model: str) -> Tuple[float]:
        """Returns the position of the model wrt the world frame using call_async."""
        if not self._gazebo_available or self.model_state_client is None:
            self.get_logger().info('Gazebo service client not available.')
            return None

        if not self.model_state_client.service_is_ready():
            self.get_logger().info('Waiting for /gazebo/get_model_state service...')
            return None

        request = self._GetModelState.Request()
        request.model_name = model
        request.relative_entity_name = 'world'

        future = self.model_state_client.call_async(request)

        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if future.done():
            response = future.result()
            if response is not None and response.success:
                return (response.pose.position.x,
                        response.pose.position.y,
                        response.pose.position.z)
            else:
                self.get_logger().warn(f'GetModelState failed for {model}')
                return None
        else:
            self.get_logger().warn(f'GetModelState timed out for {model}')
            return None

    def get_pixel_depth(self, u, v):
        if u >= self.image_width:
            u = self.image_width - 1
        if v >= self.image_height:
            v = self.image_height - 1
        return self.depth_image[int(v), int(u)]

    def compute_mass_center(self, image_array):
        cv2 = self._cv2
        M = cv2.moments(image_array)
        cx = int(M['m10'] / (M['m00'] + 1e-6))
        cy = int(M['m01'] / (M['m00'] + 1e-6))
        return cx, cy

    def get_detected_objects(self, contour_images, color):
        cv2 = self._cv2
        objects = []
        for image in contour_images:
            area = cv2.contourArea(image)
            if area > self.block_contour_area_threshold:
                cx, cy = self.compute_mass_center(image)
                _, _, w, h = cv2.boundingRect(image)
                depth = self.get_pixel_depth(cx, cy)
                height = self.workbench_depth - depth
                objects.append((cx, cy, w, h, height, color, area))
        return objects

    def get_3D_point_from_pixel(self, u, v):
        depth = self.get_pixel_depth(u, v)
        height = self.workbench_depth - depth
        ray = self.pin_cam.projectPixelTo3dRay((u, v))
        X_ray = ray[0] / ray[2]
        Y_ray = ray[1] / ray[2]
        Z = depth + height / 2
        X = X_ray * Z
        Y = Y_ray * Z
        X_world, Y_world, Z_world = self.convert_point_from_camera_to_world(X, Y, Z)
        return X_world, Y_world, Z_world

    def get_pixel_from_3D_point(self, x_world, y_world, z_world):
        x, y, z = self.convert_point_from_world_to_camera(x_world, y_world, z_world)
        u, v = self.pin_cam.project3dToPixel((x, y, z))
        return u, v

    def convert_point_from_camera_to_world(self, x, y, z):
        X_c = np.array([x, y, z, 1])
        X_world = np.dot(self.T_c_w, X_c)
        return X_world[0], X_world[1], X_world[2]

    def convert_point_from_world_to_camera(self, x_world, y_world, z_world):
        X_world = np.array([x_world, y_world, z_world, 1])
        X_c = np.dot(self.T_w_c, X_world)
        return X_c[0], X_c[1], X_c[2]

    def get_mask(self, hsv, color):
        cv2 = self._cv2
        if color not in self.color_ranges.keys():
            raise ValueError('The requested color to mask is not on the list of detectable colors.')
        mask = cv2.inRange(hsv, self.color_ranges[color][0], self.color_ranges[color][1])
        return mask

    def get_contours(self, hsv, color):
        cv2 = self._cv2
        mask = self.get_mask(hsv, color)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def get_box_dimensions(self, cx, cy, w, h):
        u1 = cx - int(w / 2)
        v1 = cy - int(h / 2)
        u2 = cx + int(w / 2)
        v2 = cy + int(h / 2)
        x1_world, y1_world, _ = self.get_3D_point_from_pixel(u1, v1)
        x2_world, y2_world, _ = self.get_3D_point_from_pixel(u2, v2)
        length = abs(x1_world - x2_world)
        width = abs(y1_world - y2_world)
        return width, length

    def image_callback(self, msg):
        if not self._initialized:
            self.try_initialize()
            if not self._initialized:
                return
        cv2 = self._cv2
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self._depth_image_data is not None:
            try:
                self.depth_image = self.bridge.imgmsg_to_cv2(self._depth_image_data, desired_encoding='32FC1')
            except Exception:
                pass
        color_contours = []
        self.blocks_on_workbench = []
        for color in ["red", "green", "blue"]:
            contours = self.get_contours(hsv, color=color)
            detected_objects = self.get_detected_objects(contours, color)
            color_contours.append(contours)
            self.blocks_on_workbench += detected_objects
        self.publish_detected_objects()

    def publish_detected_objects(self):
        blocks = DetectedObjectsStamped()
        blocks.header.stamp = self.get_clock().now().to_msg()
        blocks.detected_objects = []
        for cx, cy, w, h, height, color, area in self.blocks_on_workbench:
            X_world, Y_world, Z_world = self.get_3D_point_from_pixel(cx, cy)
            width, length = self.get_box_dimensions(cx, cy, w, h)
            detected_block = DetectedObject()
            detected_block.x_world = X_world
            detected_block.y_world = Y_world
            detected_block.z_world = Z_world
            detected_block.width = width
            detected_block.length = length
            detected_block.height = height
            detected_block.color = color
            blocks.detected_objects.append(detected_block)
        self.detected_objects_pub.publish(blocks)


def main(args=None):
    rclpy.init(args=args)
    node = VisionObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()