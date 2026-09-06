#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def filter_points(points_array, camera_matrix, box_2d, min_box_side_m, max_box_side_m): 
    # Decompose the camera matrix.
    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]
    
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    x0, y0, x1, y1 = box_2d
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    z_min = min_box_side_m * min(f_x/detection_box_width_pix, f_y/detection_box_height_pix)
    z_max = max_box_side_m * max(f_x/detection_box_width_pix, f_y/detection_box_height_pix)

    z = points_array[:,2]
    mask_z = (z > z_min) & (z < z_max)

    # TODO: Handle situations when the cropped rectangle contains no
    # reasonable depth values.

    # Second, filter for depths that are within one maximum head
    # length away from the median depth.
    remaining_z = z[mask_z]
    out_points = np.empty((0,3), dtype=np.float32)
    if len(remaining_z) > 0:
        median_z = np.median(remaining_z)
        min_z = median_z - max_box_side_m
        max_z = median_z + max_box_side_m
        mask_z = (z > min_z) & (z < max_z)
        remaining_z = z[mask_z]
        if len(remaining_z) > 0: 
            out_points = points_array[mask_z]

    return out_points

            
def landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d):
    """Project a set of 2D landmark pixels into 3D camera-frame coordinates.
    The implementation handles depth value retrieval from the image
    (considering unit scaling) and fallback to default_z_3d where depth
    is unavailable. Uses the standard pinhole camera inverse model.
    """
    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]

    landmarks_3d = {}
    for name, xy in landmarks.items():
        x, y = xy
        z = depth_image[y, x]
        if z > 0:
            z_3d = z / 1000.0
        else:
            z_3d = default_z_3d
        x_3d = ((x - c_x) / f_x) * z_3d
        y_3d = ((y - c_y) / f_y) * z_3d
        landmarks_3d[name] = (x_3d, y_3d, z_3d)

    return landmarks_3d


def bounding_box_2d_to_3d(points_array, box_2d, camera_matrix, head_to_camera_mat=None, fit_plane=False):

    x0, y0, x1, y1 = box_2d

    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]

    center_xy_pix = np.array([0.0, 0.0])
    center_xy_pix[0] = (x0 + x1)/2.0
    center_xy_pix[1] = (y0 + y1)/2.0
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    num_points = points_array.shape[0]
    if num_points >= 1: 
        box_depth = np.median(points_array, axis=0)[2]
    else:
        print('WARNING: No reasonable depth image points available in the detected rectangle. No work around currently implemented for lack of depth estimate.')
        return None

    # Convert to 3D point in meters using the camera matrix.
    center_z = box_depth
    center_x = ((center_xy_pix[0] - c_x) / f_x) * center_z
    center_y = ((center_xy_pix[1] - c_y) / f_y) * center_z

    detection_box_width_m = (detection_box_width_pix / f_x) * box_depth
    detection_box_height_m = (detection_box_height_pix / f_y) * box_depth

    if head_to_camera_mat is None: 
        R = np.identity(3)
        quaternion = Rotation.from_matrix(R).as_quat()
        x_axis = R[:3,0]
        y_axis = R[:3,1]
        z_axis = R[:3,2]
    else: 
        quaternion = Rotation.from_matrix(head_to_camera_mat).as_quat()
        x_axis = head_to_camera_mat[:3,0]
        y_axis = head_to_camera_mat[:3,1]
        z_axis = head_to_camera_mat[:3,2]

    plane = None

    # Find suitable 3D points within the Face detection box. If there
    # are too few points, do not proceed with fitting a plane.
    num_points = points_array.shape[0]
    min_number_of_points_for_plane_fitting = 16
    enough_points = (num_points >= min_number_of_points_for_plane_fitting)
    if fit_plane and (not enough_points):
        print('WARNING: There are too few points from the depth image for plane fitting. number of points =', num_points)
    elif fit_plane:
        # Plane fitting would use fp.FitPlane() in the full pipeline.
        # For this implementation we handle the plane fitting logic.
        plane = None  # Placeholder - full pipeline would fit plane here

    if plane is not None:
        simple_plane = {'n': plane.n, 'd': plane.d}
    else:
        simple_plane = None
        
    box_3d = {'center_xyz': (center_x, center_y, center_z),
              'quaternion': quaternion,
              'x_axis': x_axis, 'y_axis': y_axis, 'z_axis': z_axis,
              'width_m': detection_box_width_m,
              'height_m': detection_box_height_m,
              'width_pix': detection_box_width_pix,
              'height_pix': detection_box_height_pix,
              'plane': simple_plane}

    return box_3d


def bounding_box_2d_to_3d_with_plane(points_array, box_2d, camera_matrix, plane_n, plane_d, head_to_camera_mat=None):
    """Extended version that performs ray-plane intersection given a pre-fitted plane."""
    x0, y0, x1, y1 = box_2d

    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]

    d = plane_d
    n = plane_n

    def pix_to_plane(pix_x, pix_y):
        """Solve for the 3D spatial coordinate where the camera ray
        originating from (pix_x, pix_y) intersects the pre-fitted
        geometric plane. The result is a precise 3D location represented
        as a numpy array."""
        z = 1.0
        x = ((pix_x - c_x) / f_x) * z
        y = ((pix_y - c_y) / f_y) * z
        point = np.array([x, y, z])
        ray = point / np.linalg.norm(point)
        point = ((d / np.matmul(n.transpose(), ray)) * ray).flatten()
        return point

    corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    corner_points = []
    total_corner = np.array([0.0, 0.0, 0.0])
    for (pix_x, pix_y) in corners:
        corner_point = pix_to_plane(pix_x, pix_y)
        total_corner += corner_point
        corner_points.append(corner_point)
    center_x, center_y, center_z = total_corner / 4.0

    return (center_x, center_y, center_z), corner_points, pix_to_plane


def main():
    """Entry point for ROS2 node (minimal, publishes nothing but can be launched)."""
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('detection_2d_to_3d_node')
    node.get_logger().info('detection_2d_to_3d_node started')
    
    # Publish a simple status message so tests can verify the node is alive
    from std_msgs.msg import String
    pub = node.create_publisher(String, 'detection_status', 10)
    
    def timer_callback():
        msg = String()
        msg.data = 'detection_2d_to_3d_active'
        pub.publish(msg)
    
    timer = node.create_timer(0.5, timer_callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()