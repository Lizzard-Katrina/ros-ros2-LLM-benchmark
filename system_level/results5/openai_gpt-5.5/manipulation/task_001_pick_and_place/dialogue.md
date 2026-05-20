# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: object_detector.py
----------------------------
#!/usr/bin/env python3

"""
Detects objects based on their color and provides information 
about their position, dimensions and color.

Author: Elena Oikonomou
Date:   Fall 2023
"""

import numpy as np
import cv2, cv_bridge
import rospy
import tf.transformations

from gazebo_msgs.srv import GetModelState
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from typing import List, Tuple

from pick_and_place.msg import DetectedObjectsStamped, DetectedObject


class VisionObjectDetector:
    def __init__(self):
        
        self.color_ranges = {"blue": [np.array([110,50,50]), np.array([130,255,255])],
                             "green": [np.array([36, 25, 25]), np.array([70, 255,255])],
                             "red": [np.array([0, 100, 100]), np.array([10, 255, 255])],
                             "black": [np.array([0,0,0]), np.array([180, 255, 40])]
                             }                                              # Color ranges in HSV (Hue [0-180), Saturation [0-255], Value [0-255])
        self.block_contour_area_threshold = 200                             # Area threshold to detect blocks
        self.blocks_on_workbench = []                                       # Blocks on the workbench [(cx, cy, w, h, depth, color, area)]
        
        # TODO: Migrate ROS 1 communication and transform initializations to ROS 2.
        # - Linkage: Synchronize topic names and message types with the pick-and-place system.
        # - Style Restriction: Use 'self.declare_parameter' for configuration and 
        #   'self.create_client' for service discovery to ensure compatibility.
        # END OF TODO    
    def get_image_dimensions(self) -> Tuple[int]:
        """Computes the image height and width in pixels."""
        image = self.get_color_image()
        h, w, c = image.shape
        return h, w

    def get_camera_homogeneous_tranforms(self) -> np.ndarray:
        """Returns the homogeneous transform of the camera frame wrt world frame and its inverse.
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        """
        camera_origin = self.get_model_position_from_gazebo("kinect")        # Position of origin of camera frame wrt world frame
        Rot_c_w =  np.array([[ 0, -1,  0, 0],
                             [-1,  0,  0, 0],
                             [ 0,  0, -1, 0],
                             [ 0,  0,  0, 1]])                               # Rotation of camera wrt world (Homogeneous transform)
        Transl_c_w = tf.transformations.translation_matrix((camera_origin))  # Translation of camera wrt world (Homogeneous transform)
        T_c_w =  np.dot(Transl_c_w, Rot_c_w)                                 # Homogeneous transform of camera wrt world
        T_w_c = tf.transformations.inverse_matrix(T_c_w)                     # Homogeneous transform of world wrt camera
        return T_c_w, T_w_c
        
    def get_depth_image(self) -> np.ndarray:
        """Returns current depth image in OpenCV fomrat."""
        image_msg = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=10)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='32FC1')
        return image
    
    def get_color_image(self) -> np.ndarray:
        """Returns current color image in OpenCV fomrat."""
        image_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=10)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        return image

    def get_pinhole_camera_model(self) -> PinholeCameraModel:
        """Creates a pinhole camera model from the camera's ROS parameters."""
        pin_cam = PinholeCameraModel()
        cam_info = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=10)
        pin_cam.fromCameraInfo(cam_info)
        return pin_cam

    def get_model_position_from_gazebo(self, model: str) -> Tuple[float]:
        # TODO
        # - Linkage: Function must handle the pose data without stalling the node's executor.
        # - Style Restriction: Use the 'call_async' pattern and handle the Future object.
        # END OF TODO
    def get_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """Creates a mask of the image that detects the given color."""
        if color not in self.color_ranges.keys():
            raise ValueError('The requested color to mask is not on the list of detectable colors.')

        mask = cv2.inRange(hsv, self.color_ranges[color][0], self.color_ranges[color][1])  # Check if array elements lie between the bounds. Returns 255 if yes, 0 otherwise. 
        return mask

    def get_workbench_depth(self) -> float:
        """Computes the depth of the workbench."""

        # Detect black regions (the workbench color) in image
        image = self.get_color_image()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                                       # Convert image from BGR to HSV color-space
        mask = self.get_mask(hsv, "black")
        # All the detected black regions will be white in the mask. Loop through mask, to find a white pixel (value 255).
        height, width = mask.shape
        flag = False
        u, v = None, None
        for i in range(height):
            for j in range(width):
                if mask[i, j] == 255:
                    u, v = i, j
                    flag = True
                    break
            if flag:
                break

        # Get depth value of that pixel
        depth = self.get_pixel_depth(u, v)
        return depth

    def get_pixel_depth(self, u: float, v: float) -> float:
        """Returns the value of the pixel in the depth image."""
        if u >= self.image_width:
            u = self.image_width - 1
        if v >= self.image_height:
            v = self.image_height - 1

        return self.depth_image[int(v), int(u)]
        
    def compute_mass_center(self, image_array: np.ndarray) -> Tuple[int]:
        """Computes the center of mass of the image."""
        M = cv2.moments(image_array)
        cx = int(M['m10']/(M['m00'] + 1e-6))  # Add 1e-6 to avoid division by zero
        cy = int(M['m01']/(M['m00'] + 1e-6))
        return cx, cy

    def get_detected_objects(self, contour_images: List, color: str) -> List[Tuple]:
        """Returns all objects of the given color from the contour images.
        
        cx, cy:  Pixel coordinates of the center of mass
        w, h:    Width and height in pixels of the bounding box surounding the object
        height:  Height of object
        color:   Color of the object
        area:    Area of the object's contour
        """
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
    
    def get_3D_point_from_pixel(self, u: float, v: float) -> Tuple[float]:
        """Computes the coordinates of the 3D point in the world frame that corresponds to the given pixel.
        
        We assume the point is on an object placed on top of the workbench. 
        So, we compute the Z-axis coordinate at half of its height (center of mass).
        """

        # Get object height
        depth = self.get_pixel_depth(u, v)
        height = self.workbench_depth - depth
        
        # Get point in homogeneous coordinates wrt camera frame (up to scale)
        ray = self.pin_cam.projectPixelTo3dRay((u, v))
        
        # Convert point to cartesian coordinates wrt camera frame (up to scale) (Normalize ray so Z-component equals 1)
        X_ray = ray[0]/ray[2]
        Y_ray = ray[1]/ray[2]

        # Since we know the object's depth (therefore the point's Z coordinate wrt camera), we can compute the exact 3D point
        # Get point in cartesian coordinates wrt camera frame (x=X/Z, y=Y/Z -> X=x*Z, Y=y*Z)
        Z = depth + height/2
        X = X_ray*Z 
        Y = Y_ray*Z

        # Get point in cartesian coordinates wrt world frame
        X_world, Y_world, Z_world = self.convert_point_from_camera_to_world(X, Y, Z)

        return X_world, Y_world, Z_world
    
    def get_pixel_from_3D_point(self, x_world: float, y_world: float, z_world: float) -> Tuple[float]:
        """Computes pixel values from the 3D coordinates of a point in the world frame.

        project3dToPixel() expects the points to be wrt the camera frame defined in camera_info (here camera_link)
        """
        x, y, z = self.convert_point_from_world_to_camera(x_world, y_world, z_world)  # Point wrt camera frame
        u, v = self.pin_cam.project3dToPixel((x,y,z))
        return u, v

    def convert_point_from_camera_to_world(self, x: float, y: float, z: float) -> Tuple[float]:
        """Converts point coordinates from camera frame to world frame. 
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        (x_world, y_world, z_world = x_camera_origin - y, y_camera_origin - x, z_camera_origin - z)
        """
        X_c = np.array([x, y, z, 1])                        # Point wrt camera frame
        X_world = np.dot(self.T_c_w, X_c)                   # Point wrt world frame
        return X_world[0], X_world[1], X_world[2]
    
    def convert_point_from_world_to_camera(self, x_world: float, y_world: float, z_world: float) -> Tuple[float]:
        """Converts point coordinates from world frame to camera frame. 
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        (x, y, z = y_camera_origin - y_world, x_camera_origin - x_world, z_camera_origin - z_world)
        """
        X_world = np.array([x_world, y_world, z_world, 1])  # Point wrt world frame
        X_c = np.dot(self.T_w_c, X_world)                   # Point wrt camera frame
        return X_c[0], X_c[1], X_c[2]

    def image_callback(self, msg: Image) -> None:
        """Detects objects from images based on color and publishes relevant information."""

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert image to OpenCV fomrat
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                     # Convert image from BGR to HSV color-space.
        
        color_contours = []
        self.blocks_on_workbench = []
        for color in ["red", "green", "blue"]:
            contours = self.get_contours(hsv, color=color)
            detected_objects = self.get_detected_objects(contours, color)
            color_contours.append(contours)
            self.blocks_on_workbench += detected_objects

        # Draw bounding boxes on image
        for cx, cy, w, h, _, color, _ in self.blocks_on_workbench:
            cv2.rectangle(image, (cx-int(w/2),cy-int(h/2)), (cx+int(w/2),cy+int(h/2)), color=(255, 255, 255), thickness=1)
            cv2.putText(image, "{} ({}, {})".format(color, int(cx), int(cy)), (int(cx-45), int(cy+30)), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)

        # We could also draw the object contours (in case objects not rectangles)
        # for contours in color_contours:
        #     cv2.drawContours(image, contours, contourIdx=-1, color=(255,255,255), thickness=1)

        cv2.imshow("Camera View", image)
        cv2.waitKey(1)

        self.publish_detected_objects()
    
    def publish_detected_objects(self) -> None:
        """Publishes information about the detected objects."""

        blocks = DetectedObjectsStamped() 
        blocks.header.stamp = rospy.Time.now()
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

    def get_box_dimensions(self, cx: float, cy: float, w: float, h: float) -> Tuple[float]:
        """Computes the cartesian width and length of the box from pixel coordinates.
        
        cx, cy:  Pixel coordinates of the center of mass
        w, h:    Width and height in pixels of the bounding box surounding the object
        """
        u1 = cx - int(w/2)
        v1 = cy - int(h/2)
        u2 = cx + int(w/2)
        v2 = cy + int(h/2)

        x1_world, y1_world, _ = self.get_3D_point_from_pixel(u1, v1)
        x2_world, y2_world, _ = self.get_3D_point_from_pixel(u2, v2)
        length = abs(x1_world - x2_world)
        width = abs(y1_world - y2_world)
        return width, length

    def get_contours(self, hsv: np.ndarray, color: str) -> List:
        """Creates contours of all objects in the given color."""
        mask = self.get_mask(hsv, color)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours


if __name__ == "__main__": 
    rospy.init_node("vision_object_detector", anonymous=False)
    VisionObjectDetector()
    rospy.spin()

----------------------------

FILE_PATH: pick_and_place_state_machine.py
----------------------------
#!/usr/bin/env python3

"""
A state machine for pick-and-place tasks.

The robot starts in the home position. If objects are detected on the workbench,
it randomly selects one. Once an object is selected, the robot picks and places it to 
the same color bin. Then, returns to the home position. If no objects are 
detected on the workbench, it stops.

Author: Elena Oikonomou
Date:   Fall 2023
"""

import rospy
import os
import rospkg
from statemachine import State, StateMachine
from statemachine.contrib.diagram import DotGraphMachine

from controller import Controller


class PickAndPlaceStateMachine(StateMachine):

    # States
    home = State("Home", initial=True)
    selecting_object = State("SelectingObject")                       
    picking_and_placing = State("PickingAndPlacing")                         
    done = State("Done", final=True)

    # Events & Transitions
    select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
    pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
    get_ready = picking_and_placing.to(home, cond="object_placed")
    
    def __init__(self, controller):
        self.controller = controller
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        print('\n' + 80*'=')
        rospy.loginfo("*** Pick-and-Place Mission Begins! ***") 
        print(80*'=')

        super().__init__()

    def are_objects_detected(self) -> bool:
        """Guard to transition to 'selecting_object' state when in 'home' state."""
        return self.controller.are_objects_on_workbench()

    # Actions that occur when entering states
    def on_enter_home(self) -> None:
        """Moves robot to home position and triggers the 'select_object' event."""
        rospy.loginfo("Moving to home position..") 
        self.controller.panda.move_to_neutral()
        self.object_selected = False
        self.object_placed = False
        rospy.sleep(0.2)

        self.send("select_object")

    def on_enter_selecting_object(self) -> None:
        # TODO 
        # State transitions must be tethered to the actual completion of the controller's task. - Style 
        # Restriction: Implement a mechanism that waits for a 'Future' (callback or spin) before triggering 
        # END OF TODO
    def on_enter_picking_and_placing(self) -> None:
        """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
        rospy.loginfo("Starting pick & place operation of selected object..") 
        self.controller.move_object(self.currently_selected_object)
        self.object_placed = True
        rospy.loginfo("Object placed in its bin.")

        self.send("get_ready")
    
    def on_enter_done(self) -> None:
        """Reports that the robot has finished its pick-and-place tasks."""
        print('\n' + 60*'=')
        rospy.loginfo("*** Mission Complete! *** \nAll objects have been placed in their bins.") 
        print(60*'=')
        exit()


def create_state_machine_graph():
    """Creates and saves an image of the state machine graph."""
    graph = DotGraphMachine(PickAndPlaceStateMachine)
    dot = graph()
    file_path = os.path.join(rospkg.RosPack().get_path('pick_and_place'), 'images/')
    dot.write_png(file_path + "state_machine.png")
    print("\n State machine image saved! \n")


if __name__ == "__main__":     
    
    # TODO
    # - Linkage: Link the StateMachine and the Controller within the same execution context.
    # - Style Restriction: Use a ROS 2 'Executor' (e.g. MultiThreadedExecutor) to manage the node lifecycles.
    # END OF TODO

----------------------------


# LLM Response

[FILENAME: object_detector.py]
#!/usr/bin/env python3

"""
Detects objects based on their color and provides information 
about their position, dimensions and color.

Author: Elena Oikonomou
Date:   Fall 2023
"""

import numpy as np
import cv2, cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
import tf_transformations as tf_transformations

from gazebo_msgs.srv import GetModelState
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from typing import List, Tuple

from pick_and_place.msg import DetectedObjectsStamped, DetectedObject


class VisionObjectDetector(Node):
    def __init__(self):
        super().__init__("vision_object_detector")
        
        self.color_ranges = {"blue": [np.array([110,50,50]), np.array([130,255,255])],
                             "green": [np.array([36, 25, 25]), np.array([70, 255,255])],
                             "red": [np.array([0, 100, 100]), np.array([10, 255, 255])],
                             "black": [np.array([0,0,0]), np.array([180, 255, 40])]
                             }                                              # Color ranges in HSV (Hue [0-180), Saturation [0-255], Value [0-255])
        self.block_contour_area_threshold = 200                             # Area threshold to detect blocks
        self.blocks_on_workbench = []                                       # Blocks on the workbench [(cx, cy, w, h, depth, color, area)]
        
        self.declare_parameter("color_image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("detected_objects_topic", "/detected_objects")
        self.declare_parameter("gazebo_model_state_service", "/gazebo/get_model_state")
        self.declare_parameter("camera_model_name", "kinect")

        self.color_image_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value
        self.depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.detected_objects_topic = self.get_parameter("detected_objects_topic").get_parameter_value().string_value
        self.gazebo_model_state_service = self.get_parameter("gazebo_model_state_service").get_parameter_value().string_value
        self.camera_model_name = self.get_parameter("camera_model_name").get_parameter_value().string_value

        self.bridge = cv_bridge.CvBridge()
        self._last_color_image_msg = None
        self._last_depth_image_msg = None
        self._last_camera_info_msg = None

        self.detected_objects_pub = self.create_publisher(
            DetectedObjectsStamped,
            self.detected_objects_topic,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            self.color_image_topic,
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_image_topic,
            self._depth_image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10
        )

        self.get_model_state_client = self.create_client(GetModelState, self.gazebo_model_state_service)
        while rclpy.ok() and not self.get_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Gazebo model state service...")

        self.depth_image = None
        self.pin_cam = None
        self.image_height = 0
        self.image_width = 0
        self.workbench_depth = 0.0
        self.T_c_w = np.identity(4)
        self.T_w_c = np.identity(4)

        try:
            self.depth_image = self.get_depth_image()
            self.image_height, self.image_width = self.get_image_dimensions()
            self.pin_cam = self.get_pinhole_camera_model()
            self.T_c_w, self.T_w_c = self.get_camera_homogeneous_tranforms()
            self.workbench_depth = self.get_workbench_depth()
        except Exception as exc:
            self.get_logger().warn("Initial vision calibration deferred: {}".format(exc))
    
    def _depth_image_callback(self, msg: Image) -> None:
        self._last_depth_image_msg = msg
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn("Failed to convert depth image: {}".format(exc))

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._last_camera_info_msg = msg
        if self.pin_cam is None:
            self.pin_cam = PinholeCameraModel()
        self.pin_cam.fromCameraInfo(msg)

    def _wait_for_message(self, attr_name: str, timeout_sec: float = 10.0):
        start = self.get_clock().now()
        while rclpy.ok():
            msg = getattr(self, attr_name)
            if msg is not None:
                return msg
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - start) > Duration(seconds=timeout_sec):
                raise TimeoutError("Timed out waiting for {}".format(attr_name))
        raise RuntimeError("ROS shutdown while waiting for {}".format(attr_name))

    def get_image_dimensions(self) -> Tuple[int]:
        """Computes the image height and width in pixels."""
        image = self.get_color_image()
        h, w, c = image.shape
        return h, w

    def get_camera_homogeneous_tranforms(self) -> np.ndarray:
        """Returns the homogeneous transform of the camera frame wrt world frame and its inverse.
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        """
        camera_origin = self.get_model_position_from_gazebo("kinect")        # Position of origin of camera frame wrt world frame
        Rot_c_w =  np.array([[ 0, -1,  0, 0],
                             [-1,  0,  0, 0],
                             [ 0,  0, -1, 0],
                             [ 0,  0,  0, 1]])                               # Rotation of camera wrt world (Homogeneous transform)
        Transl_c_w = tf_transformations.translation_matrix((camera_origin))  # Translation of camera wrt world (Homogeneous transform)
        T_c_w =  np.dot(Transl_c_w, Rot_c_w)                                 # Homogeneous transform of camera wrt world
        T_w_c = tf_transformations.inverse_matrix(T_c_w)                     # Homogeneous transform of world wrt camera
        return T_c_w, T_w_c
        
    def get_depth_image(self) -> np.ndarray:
        """Returns current depth image in OpenCV fomrat."""
        image_msg = self._wait_for_message('_last_depth_image_msg', timeout_sec=10)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='32FC1')
        self.depth_image = image
        return image
    
    def get_color_image(self) -> np.ndarray:
        """Returns current color image in OpenCV fomrat."""
        image_msg = self._wait_for_message('_last_color_image_msg', timeout_sec=10)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        return image

    def get_pinhole_camera_model(self) -> PinholeCameraModel:
        """Creates a pinhole camera model from the camera's ROS parameters."""
        pin_cam = PinholeCameraModel()
        cam_info = self._wait_for_message('_last_camera_info_msg', timeout_sec=10)
        pin_cam.fromCameraInfo(cam_info)
        self.pin_cam = pin_cam
        return pin_cam

    def get_model_position_from_gazebo(self, model: str) -> Tuple[float]:
        if not self.get_model_state_client.service_is_ready():
            self.get_model_state_client.wait_for_service(timeout_sec=5.0)

        request = GetModelState.Request()
        request.model_name = model
        request.relative_entity_name = "world"

        future = self.get_model_state_client.call_async(request)
        start = self.get_clock().now()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - start) > Duration(seconds=10.0):
                raise TimeoutError("Timed out waiting for Gazebo model state for '{}'".format(model))

        response = future.result()
        if response is None:
            raise RuntimeError("Failed to call Gazebo model state service for '{}'".format(model))
        if hasattr(response, "success") and not response.success:
            raise RuntimeError("Gazebo model state request failed for '{}': {}".format(model, response.status_message))

        position = response.pose.position
        return position.x, position.y, position.z

    def get_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """Creates a mask of the image that detects the given color."""
        if color not in self.color_ranges.keys():
            raise ValueError('The requested color to mask is not on the list of detectable colors.')

        mask = cv2.inRange(hsv, self.color_ranges[color][0], self.color_ranges[color][1])  # Check if array elements lie between the bounds. Returns 255 if yes, 0 otherwise. 
        return mask

    def get_workbench_depth(self) -> float:
        """Computes the depth of the workbench."""

        # Detect black regions (the workbench color) in image
        image = self.get_color_image()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                                       # Convert image from BGR to HSV color-space
        mask = self.get_mask(hsv, "black")
        # All the detected black regions will be white in the mask. Loop through mask, to find a white pixel (value 255).
        height, width = mask.shape
        flag = False
        u, v = None, None
        for i in range(height):
            for j in range(width):
                if mask[i, j] == 255:
                    u, v = i, j
                    flag = True
                    break
            if flag:
                break

        # Get depth value of that pixel
        depth = self.get_pixel_depth(u, v)
        return depth

    def get_pixel_depth(self, u: float, v: float) -> float:
        """Returns the value of the pixel in the depth image."""
        if self.depth_image is None:
            self.depth_image = self.get_depth_image()
        if self.image_width == 0 or self.image_height == 0:
            self.image_height, self.image_width = self.depth_image.shape[:2]

        if u >= self.image_width:
            u = self.image_width - 1
        if v >= self.image_height:
            v = self.image_height - 1
        if u < 0:
            u = 0
        if v < 0:
            v = 0

        return self.depth_image[int(v), int(u)]
        
    def compute_mass_center(self, image_array: np.ndarray) -> Tuple[int]:
        """Computes the center of mass of the image."""
        M = cv2.moments(image_array)
        cx = int(M['m10']/(M['m00'] + 1e-6))  # Add 1e-6 to avoid division by zero
        cy = int(M['m01']/(M['m00'] + 1e-6))
        return cx, cy

    def get_detected_objects(self, contour_images: List, color: str) -> List[Tuple]:
        """Returns all objects of the given color from the contour images.
        
        cx, cy:  Pixel coordinates of the center of mass
        w, h:    Width and height in pixels of the bounding box surounding the object
        height:  Height of object
        color:   Color of the object
        area:    Area of the object's contour
        """
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
    
    def get_3D_point_from_pixel(self, u: float, v: float) -> Tuple[float]:
        """Computes the coordinates of the 3D point in the world frame that corresponds to the given pixel.
        
        We assume the point is on an object placed on top of the workbench. 
        So, we compute the Z-axis coordinate at half of its height (center of mass).
        """

        # Get object height
        depth = self.get_pixel_depth(u, v)
        height = self.workbench_depth - depth
        
        # Get point in homogeneous coordinates wrt camera frame (up to scale)
        if self.pin_cam is None:
            self.pin_cam = self.get_pinhole_camera_model()
        ray = self.pin_cam.projectPixelTo3dRay((u, v))
        
        # Convert point to cartesian coordinates wrt camera frame (up to scale) (Normalize ray so Z-component equals 1)
        X_ray = ray[0]/ray[2]
        Y_ray = ray[1]/ray[2]

        # Since we know the object's depth (therefore the point's Z coordinate wrt camera), we can compute the exact 3D point
        # Get point in cartesian coordinates wrt camera frame (x=X/Z, y=Y/Z -> X=x*Z, Y=y*Z)
        Z = depth + height/2
        X = X_ray*Z 
        Y = Y_ray*Z

        # Get point in cartesian coordinates wrt world frame
        X_world, Y_world, Z_world = self.convert_point_from_camera_to_world(X, Y, Z)

        return X_world, Y_world, Z_world
    
    def get_pixel_from_3D_point(self, x_world: float, y_world: float, z_world: float) -> Tuple[float]:
        """Computes pixel values from the 3D coordinates of a point in the world frame.

        project3dToPixel() expects the points to be wrt the camera frame defined in camera_info (here camera_link)
        """
        x, y, z = self.convert_point_from_world_to_camera(x_world, y_world, z_world)  # Point wrt camera frame
        if self.pin_cam is None:
            self.pin_cam = self.get_pinhole_camera_model()
        u, v = self.pin_cam.project3dToPixel((x,y,z))
        return u, v

    def convert_point_from_camera_to_world(self, x: float, y: float, z: float) -> Tuple[float]:
        """Converts point coordinates from camera frame to world frame. 
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        (x_world, y_world, z_world = x_camera_origin - y, y_camera_origin - x, z_camera_origin - z)
        """
        X_c = np.array([x, y, z, 1])                        # Point wrt camera frame
        X_world = np.dot(self.T_c_w, X_c)                   # Point wrt world frame
        return X_world[0], X_world[1], X_world[2]
    
    def convert_point_from_world_to_camera(self, x_world: float, y_world: float, z_world: float) -> Tuple[float]:
        """Converts point coordinates from world frame to camera frame. 
        
        The camera frame is following the opencv convention (not the frame of the camera model in Gazebo) 
        where the Z-axis is along the optical axis.
        (x, y, z = y_camera_origin - y_world, x_camera_origin - x_world, z_camera_origin - z_world)
        """
        X_world = np.array([x_world, y_world, z_world, 1])  # Point wrt world frame
        X_c = np.dot(self.T_w_c, X_world)                   # Point wrt camera frame
        return X_c[0], X_c[1], X_c[2]

    def image_callback(self, msg: Image) -> None:
        """Detects objects from images based on color and publishes relevant information."""

        self._last_color_image_msg = msg
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert image to OpenCV fomrat
        if self.image_height == 0 or self.image_width == 0:
            self.image_height, self.image_width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                     # Convert image from BGR to HSV color-space.
        
        color_contours = []
        self.blocks_on_workbench = []
        for color in ["red", "green", "blue"]:
            contours = self.get_contours(hsv, color=color)
            detected_objects = self.get_detected_objects(contours, color)
            color_contours.append(contours)
            self.blocks_on_workbench += detected_objects

        # Draw bounding boxes on image
        for cx, cy, w, h, _, color, _ in self.blocks_on_workbench:
            cv2.rectangle(image, (cx-int(w/2),cy-int(h/2)), (cx+int(w/2),cy+int(h/2)), color=(255, 255, 255), thickness=1)
            cv2.putText(image, "{} ({}, {})".format(color, int(cx), int(cy)), (int(cx-45), int(cy+30)), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)

        # We could also draw the object contours (in case objects not rectangles)
        # for contours in color_contours:
        #     cv2.drawContours(image, contours, contourIdx=-1, color=(255,255,255), thickness=1)

        cv2.imshow("Camera View", image)
        cv2.waitKey(1)

        self.publish_detected_objects()
    
    def publish_detected_objects(self) -> None:
        """Publishes information about the detected objects."""

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

    def get_box_dimensions(self, cx: float, cy: float, w: float, h: float) -> Tuple[float]:
        """Computes the cartesian width and length of the box from pixel coordinates.
        
        cx, cy:  Pixel coordinates of the center of mass
        w, h:    Width and height in pixels of the bounding box surounding the object
        """
        u1 = cx - int(w/2)
        v1 = cy - int(h/2)
        u2 = cx + int(w/2)
        v2 = cy + int(h/2)

        x1_world, y1_world, _ = self.get_3D_point_from_pixel(u1, v1)
        x2_world, y2_world, _ = self.get_3D_point_from_pixel(u2, v2)
        length = abs(x1_world - x2_world)
        width = abs(y1_world - y2_world)
        return width, length

    def get_contours(self, hsv: np.ndarray, color: str) -> List:
        """Creates contours of all objects in the given color."""
        mask = self.get_mask(hsv, color)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours


if __name__ == "__main__": 
    rclpy.init()
    node = VisionObjectDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

[FILENAME: pick_and_place_state_machine.py]
#!/usr/bin/env python3

"""
A state machine for pick-and-place tasks.

The robot starts in the home position. If objects are detected on the workbench,
it randomly selects one. Once an object is selected, the robot picks and places it to 
the same color bin. Then, returns to the home position. If no objects are 
detected on the workbench, it stops.

Author: Elena Oikonomou
Date:   Fall 2023
"""

import rclpy
import os
import threading
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from statemachine import State, StateMachine
from statemachine.contrib.diagram import DotGraphMachine

from controller import Controller


class PickAndPlaceStateMachine(StateMachine):

    # States
    home = State("Home", initial=True)
    selecting_object = State("SelectingObject")                       
    picking_and_placing = State("PickingAndPlacing")                         
    done = State("Done", final=True)

    # Events & Transitions
    select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
    pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
    get_ready = picking_and_placing.to(home, cond="object_placed")
    
    def __init__(self, controller):
        self.controller = controller
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        print('\n' + 80*'=')
        self.controller.get_logger().info("*** Pick-and-Place Mission Begins! ***") 
        print(80*'=')

        super().__init__()

    def are_objects_detected(self) -> bool:
        """Guard to transition to 'selecting_object' state when in 'home' state."""
        return self.controller.are_objects_on_workbench()

    # Actions that occur when entering states
    def on_enter_home(self) -> None:
        """Moves robot to home position and triggers the 'select_object' event."""
        self.controller.get_logger().info("Moving to home position..") 
        self.controller.panda.move_to_neutral()
        self.object_selected = False
        self.object_placed = False

        future = rclpy.task.Future()
        timer_holder = {}

        def complete_home_wait():
            if not future.done():
                future.set_result(True)
            timer_holder["timer"].cancel()

        timer_holder["timer"] = self.controller.create_timer(0.2, complete_home_wait)
        future.add_done_callback(lambda _: self.send("select_object"))
    
    def on_enter_selecting_object(self) -> None:
        self.controller.get_logger().info("Selecting an object from the workbench..")

        selection_future = None
        if hasattr(self.controller, "select_object_async"):
            selection_future = self.controller.select_object_async()
        elif hasattr(self.controller, "select_object"):
            result = self.controller.select_object()
            if isinstance(result, rclpy.task.Future) or hasattr(result, "add_done_callback"):
                selection_future = result
            else:
                selection_future = rclpy.task.Future()
                selection_future.set_result(result)
        elif hasattr(self.controller, "select_random_object"):
            result = self.controller.select_random_object()
            if isinstance(result, rclpy.task.Future) or hasattr(result, "add_done_callback"):
                selection_future = result
            else:
                selection_future = rclpy.task.Future()
                selection_future.set_result(result)
        else:
            selection_future = rclpy.task.Future()
            selection_future.set_exception(RuntimeError("Controller has no object selection method"))

        def selection_done(future):
            try:
                selected_object = future.result()
            except Exception as exc:
                self.controller.get_logger().error("Object selection failed: {}".format(exc))
                selected_object = None

            self.currently_selected_object = selected_object
            self.object_selected = selected_object is not None

            if self.object_selected:
                color = getattr(selected_object, "color", "unknown")
                self.controller.get_logger().info("Selected {} object.".format(color))
                self.send("pick_object")
            else:
                self.controller.get_logger().info("No object could be selected.")
                self.send("select_object")

        selection_future.add_done_callback(selection_done)

    def on_enter_picking_and_placing(self) -> None:
        """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
        self.controller.get_logger().info("Starting pick & place operation of selected object..") 
        result = self.controller.move_object(self.currently_selected_object)

        if isinstance(result, rclpy.task.Future) or hasattr(result, "add_done_callback"):
            future = result
        else:
            future = rclpy.task.Future()
            future.set_result(result)

        def move_done(future):
            try:
                future.result()
                self.object_placed = True
                self.controller.get_logger().info("Object placed in its bin.")
            except Exception as exc:
                self.object_placed = False
                self.controller.get_logger().error("Pick & place operation failed: {}".format(exc))
            self.send("get_ready")

        future.add_done_callback(move_done)
    
    def on_enter_done(self) -> None:
        """Reports that the robot has finished its pick-and-place tasks."""
        print('\n' + 60*'=')
        self.controller.get_logger().info("*** Mission Complete! *** \nAll objects have been placed in their bins.") 
        print(60*'=')
        rclpy.shutdown()


def create_state_machine_graph():
    """Creates and saves an image of the state machine graph."""
    graph = DotGraphMachine(PickAndPlaceStateMachine)
    dot = graph()
    file_path = os.path.join(get_package_share_directory('pick_and_place'), 'images/')
    os.makedirs(file_path, exist_ok=True)
    dot.write_png(file_path + "state_machine.png")
    print("\n State machine image saved! \n")


if __name__ == "__main__":     
    rclpy.init()

    controller = Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        state_machine = PickAndPlaceStateMachine(controller)
        state_machine.send("select_object")
        while rclpy.ok():
            pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)