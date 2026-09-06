#!/usr/bin/env python3
"""
Runtime tests for the Stretch 3D perception pipeline.
Tests the actual mathematical functions with concrete values.
Also tests the ROS2 node can launch and publish.
"""

import pytest
import numpy as np
import subprocess
import time
import sys
import os

# Add the package root to path so we can import detection_2d_to_3d
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class TestLandmarks2dTo3d:
    """Test the landmarks_2d_to_3d function with concrete values."""

    def test_basic_projection_with_valid_depth(self):
        from detection_2d_to_3d import landmarks_2d_to_3d

        # Camera matrix: fx=500, fy=500, cx=320, cy=240
        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Depth image with known depth at pixel (320, 240) = 2000mm = 2.0m
        depth_image = np.zeros((480, 640), dtype=np.uint16)
        depth_image[240, 320] = 2000  # 2000mm

        landmarks = {'center': (320, 240)}
        default_z_3d = 1.0

        result = landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d)

        assert 'center' in result
        x_3d, y_3d, z_3d = result['center']

        # At the principal point (cx, cy), x_3d and y_3d should be 0
        assert abs(z_3d - 2.0) < 1e-6, f"Expected z=2.0, got {z_3d}"
        assert abs(x_3d) < 1e-6, f"Expected x=0.0, got {x_3d}"
        assert abs(y_3d) < 1e-6, f"Expected y=0.0, got {y_3d}"

    def test_projection_off_center(self):
        from detection_2d_to_3d import landmarks_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        depth_image = np.zeros((480, 640), dtype=np.uint16)
        depth_image[240, 420] = 3000  # 3000mm = 3.0m at pixel (420, 240)

        landmarks = {'point': (420, 240)}
        default_z_3d = 1.0

        result = landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d)

        x_3d, y_3d, z_3d = result['point']

        # z should be 3.0m
        assert abs(z_3d - 3.0) < 1e-6, f"Expected z=3.0, got {z_3d}"
        # x = ((420 - 320) / 500) * 3.0 = (100/500)*3.0 = 0.6
        assert abs(x_3d - 0.6) < 1e-6, f"Expected x=0.6, got {x_3d}"
        # y = ((240 - 240) / 500) * 3.0 = 0.0
        assert abs(y_3d) < 1e-6, f"Expected y=0.0, got {y_3d}"

    def test_fallback_to_default_depth(self):
        from detection_2d_to_3d import landmarks_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Depth image with zero depth (invalid)
        depth_image = np.zeros((480, 640), dtype=np.uint16)

        landmarks = {'point': (320, 240)}
        default_z_3d = 1.5

        result = landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d)

        x_3d, y_3d, z_3d = result['point']
        # Should fall back to default_z_3d = 1.5
        assert abs(z_3d - 1.5) < 1e-6, f"Expected z=1.5 (default), got {z_3d}"

    def test_depth_unit_scaling_mm_to_m(self):
        """Verify that depth values are converted from mm to meters."""
        from detection_2d_to_3d import landmarks_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        depth_image = np.zeros((480, 640), dtype=np.uint16)
        depth_image[100, 200] = 5000  # 5000mm = 5.0m

        landmarks = {'test': (200, 100)}
        default_z_3d = 1.0

        result = landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d)
        _, _, z_3d = result['test']

        # Must be 5.0 meters, not 5000
        assert abs(z_3d - 5.0) < 1e-6, f"Depth not properly scaled: got {z_3d}"


class TestBoundingBox2dTo3d:
    """Test bounding_box_2d_to_3d with concrete values."""

    def test_basic_box_projection(self):
        from detection_2d_to_3d import bounding_box_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Create points array with known depth
        points = np.array([
            [0.0, 0.0, 2.0],
            [0.1, 0.0, 2.0],
            [-0.1, 0.0, 2.0],
            [0.0, 0.1, 2.0],
        ], dtype=np.float32)

        box_2d = (270, 190, 370, 290)  # 100x100 pixel box centered at (320, 240)

        result = bounding_box_2d_to_3d(points, box_2d, camera_matrix)

        assert result is not None
        assert 'center_xyz' in result
        cx, cy, cz = result['center_xyz']
        # Median depth should be 2.0
        assert abs(cz - 2.0) < 1e-6, f"Expected center_z=2.0, got {cz}"

    def test_median_depth_used(self):
        """Verify np.median is used for robust depth estimation."""
        from detection_2d_to_3d import bounding_box_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Points with outlier
        points = np.array([
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 100.0],  # outlier
            [0.0, 0.0, 2.0],
        ], dtype=np.float32)

        box_2d = (270, 190, 370, 290)

        result = bounding_box_2d_to_3d(points, box_2d, camera_matrix)
        assert result is not None
        _, _, cz = result['center_xyz']
        # Median of [2, 2, 2, 100, 2] = 2.0
        assert abs(cz - 2.0) < 1e-6, f"Expected median depth=2.0, got {cz}"

    def test_empty_points_returns_none(self):
        from detection_2d_to_3d import bounding_box_2d_to_3d

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        points = np.empty((0, 3), dtype=np.float32)
        box_2d = (270, 190, 370, 290)

        result = bounding_box_2d_to_3d(points, box_2d, camera_matrix)
        assert result is None


class TestRayPlaneIntersection:
    """Test the ray-plane intersection logic."""

    def test_pix_to_plane_basic(self):
        from detection_2d_to_3d import bounding_box_2d_to_3d_with_plane

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Plane at z=3.0: normal = [0, 0, 1], d = 3.0
        plane_n = np.array([[0.0], [0.0], [1.0]])
        plane_d = 3.0

        points = np.array([
            [0.0, 0.0, 3.0],
        ] * 20, dtype=np.float32)

        box_2d = (270, 190, 370, 290)

        center, corner_points, pix_to_plane = bounding_box_2d_to_3d_with_plane(
            points, box_2d, camera_matrix, plane_n, plane_d
        )

        # Test that pix_to_plane at the principal point gives (0, 0, 3)
        result = pix_to_plane(320, 240)
        assert abs(result[2] - 3.0) < 1e-4, f"Expected z=3.0, got {result[2]}"
        assert abs(result[0]) < 1e-4, f"Expected x≈0, got {result[0]}"
        assert abs(result[1]) < 1e-4, f"Expected y≈0, got {result[1]}"

    def test_pix_to_plane_off_center(self):
        from detection_2d_to_3d import bounding_box_2d_to_3d_with_plane

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Plane at z=5.0
        plane_n = np.array([[0.0], [0.0], [1.0]])
        plane_d = 5.0

        points = np.array([[0.0, 0.0, 5.0]] * 20, dtype=np.float32)
        box_2d = (270, 190, 370, 290)

        center, corner_points, pix_to_plane = bounding_box_2d_to_3d_with_plane(
            points, box_2d, camera_matrix, plane_n, plane_d
        )

        # At pixel (420, 240): ray direction ~ ((420-320)/500, 0, 1) normalized
        # Intersection with z=5 plane should give x = ((420-320)/500)*5 = 1.0
        result = pix_to_plane(420, 240)
        # The z component should be 5.0
        assert abs(result[2] - 5.0) < 0.1, f"Expected z≈5.0, got {result[2]}"
        # x should be approximately 1.0
        assert abs(result[0] - 1.0) < 0.1, f"Expected x≈1.0, got {result[0]}"

    def test_result_is_flattened(self):
        """Verify the result from pix_to_plane is a 1D array."""
        from detection_2d_to_3d import bounding_box_2d_to_3d_with_plane

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        plane_n = np.array([[0.0], [0.0], [1.0]])
        plane_d = 3.0

        points = np.array([[0.0, 0.0, 3.0]] * 20, dtype=np.float32)
        box_2d = (270, 190, 370, 290)

        center, corner_points, pix_to_plane = bounding_box_2d_to_3d_with_plane(
            points, box_2d, camera_matrix, plane_n, plane_d
        )

        result = pix_to_plane(320, 240)
        assert result.ndim == 1, f"Expected 1D array, got {result.ndim}D"
        assert len(result) == 3, f"Expected 3 elements, got {len(result)}"


class TestFilterPoints:
    """Test the filter_points function."""

    def test_filter_removes_outliers(self):
        from detection_2d_to_3d import filter_points

        camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Points at reasonable depth (2m) and one outlier at 100m
        points = np.array([
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 2.1],
            [0.0, 0.0, 1.9],
            [0.0, 0.0, 100.0],  # outlier
        ], dtype=np.float32)

        box_2d = (270, 190, 370, 290)
        min_box_side_m = 0.08
        max_box_side_m = 0.4

        result = filter_points(points, camera_matrix, box_2d, min_box_side_m, max_box_side_m)
        # The outlier at 100m should be filtered out
        if len(result) > 0:
            assert all(result[:, 2] < 50.0), "Outlier points should be filtered"


class TestROS2NodeLaunch:
    """Test that the ROS2 node can launch and publish."""

    def test_node_publishes_status(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        # Launch the node as a subprocess
        proc = subprocess.Popen(
            [sys.executable, os.path.join(os.path.dirname(__file__), 'detection_2d_to_3d.py')],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        received_msgs = []

        try:
            rclpy.init()
            test_node = rclpy.create_node('test_detection_listener')

            def callback(msg):
                received_msgs.append(msg.data)

            sub = test_node.create_subscription(
                String, 'detection_status', callback, 10
            )

            start_time = time.time()
            timeout = 8.0

            while time.time() - start_time < timeout:
                rclpy.spin_once(test_node, timeout_sec=0.5)
                if len(received_msgs) > 0:
                    break

            assert len(received_msgs) > 0, "No status messages received from node"
            assert received_msgs[0] == 'detection_2d_to_3d_active', \
                f"Unexpected message: {received_msgs[0]}"

        finally:
            test_node.destroy_node()
            rclpy.shutdown()
            proc.terminate()
            proc.wait(timeout=5)