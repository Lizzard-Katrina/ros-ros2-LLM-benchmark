#!/usr/bin/env python3
"""
Runtime test for task_003_rgbd_pointcloud_generation.

Since this is a C++ source file that plugs into the existing depth_image_proc
package (it implements PointCloudXyzrgbNode which is already declared in
depth_image_proc headers), we:

1. Verify the source file exists and contains the required filled-in code.
2. Actually launch the real depth_image_proc PointCloudXyzrgbNode component,
   feed it synthetic depth + RGB + camera_info messages, and verify a
   PointCloud2 is produced with correct structure.
"""

import os
import re
import time
import subprocess
import signal

import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np


def find_source_file():
    """Find the point_cloud_xyzrgb.cpp file."""
    candidates = [
        os.path.join(os.path.dirname(__file__), 'point_cloud_xyzrgb.cpp'),
        os.path.join(os.path.dirname(__file__), 'src', 'point_cloud_xyzrgb.cpp'),
    ]
    try:
        import subprocess as sp
        result = sp.run(
            ['find', '/opt/ros', '-name', 'point_cloud_xyzrgb.cpp', '-path',
             '*/task_003*'],
            capture_output=True, text=True, timeout=5)
        for line in result.stdout.strip().split('\n'):
            if line:
                candidates.append(line)
    except Exception:
        pass

    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ament_prefix.split(':'):
        p = os.path.join(prefix, 'share', 'task_003_rgbd_pointcloud_generation',
                         'point_cloud_xyzrgb.cpp')
        candidates.append(p)

    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def get_source_content():
    """Read and return the source file content with comments stripped."""
    path = find_source_file()
    assert path is not None, "Could not find point_cloud_xyzrgb.cpp"
    with open(path, 'r') as f:
        content = f.read()
    stripped = re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.DOTALL)
    return content, stripped


class TestSourceFileContent:
    """Verify the translated source file has the required patterns."""

    def setup_method(self):
        self.raw_content, self.content = get_source_content()

    def test_intrinsic_scaling_math(self):
        """Must scale focal lengths (fx/fy) when resizing."""
        scaling_pattern = r"(?:\.k|k|p|P|fx|fy|K)\[?[0-9]?\]?\s*\*=\s*ratio"
        assert re.search(scaling_pattern, self.content), \
            "Scaling Defect: didn't scale focal length"

    def test_offset_variable_usage(self):
        """Must define red_offset and blue_offset."""
        assert "red_offset" in self.content and "blue_offset" in self.content

    def test_mandatory_kernel_call(self):
        """Must use convertDepth and convertRgb kernels."""
        assert "convertDepth" in self.content and "convertRgb" in self.content

    def test_header_and_frame_sync(self):
        """Frame_id must come from depth sensor."""
        assert re.search(r"cloud_msg->header\s*=\s*depth_msg->header", self.content)

    def test_memory_unique_ownership(self):
        """Use unique_ptr and move semantics."""
        assert "std::make_unique" in self.content
        assert "std::move" in self.content

    def test_dispatch_logic_16uc1_32fc1(self):
        """Must handle both depth pixel formats."""
        assert "TYPE_16UC1" in self.content and "TYPE_32FC1" in self.content


class TestRuntimePointCloud:
    """
    Launch the real depth_image_proc PointCloudXyzrgbNode, publish synthetic
    data, and verify PointCloud2 output.
    """

    def test_pointcloud_generation(self):
        rclpy.init()
        node = None
        proc = None
        received_clouds = []

        try:
            node = rclpy.create_node('test_xyzrgb_runtime')

            # Use a QoS that is compatible with SystemDefaultsQoS publisher.
            # The depth_image_proc node publishes with SystemDefaultsQoS.
            # We use BEST_EFFORT reliability on the subscriber side to be
            # compatible with both RELIABLE and BEST_EFFORT publishers.
            sub_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )

            # For publishers, use RELIABLE to match what the node's subscribers expect
            # (SystemDefaultsQoS = RELIABLE)
            pub_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )

            # Publishers for depth, rgb, camera_info
            depth_pub = node.create_publisher(
                Image,
                '/depth_registered/image_rect',
                pub_qos
            )
            rgb_pub = node.create_publisher(
                Image,
                '/rgb/image_rect_color',
                pub_qos
            )
            info_pub = node.create_publisher(
                CameraInfo,
                '/rgb/camera_info',
                pub_qos
            )

            # Subscriber for output - use BEST_EFFORT to be compatible
            def cloud_cb(msg):
                received_clouds.append(msg)

            cloud_sub = node.create_subscription(
                PointCloud2,
                '/points',
                cloud_cb,
                sub_qos
            )

            # Launch the node using ros2 component standalone
            # The point_cloud_xyzrgb_node executable may not exist as a standalone;
            # use ros2 run with component_container + load, or try the standalone node.
            # First try: use ros2 run depth_image_proc point_cloud_xyzrgb_node
            # If that doesn't exist, use component container approach.

            # Try launching as a composable node in a component container
            proc = subprocess.Popen(
                [
                    'ros2', 'run', 'rclcpp_components', 'component_container',
                    '--ros-args', '-r', '__node:=my_container',
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Give container time to start
            time.sleep(2.0)

            # Load the component
            load_proc = subprocess.Popen(
                [
                    'ros2', 'component', 'load', '/my_container',
                    'depth_image_proc', 'depth_image_proc::PointCloudXyzrgbNode',
                    '--node-name', 'point_cloud_xyzrgb_node',
                    '-p', 'queue_size:=10',
                    '-p', 'exact_sync:=true',
                    # Override the points publisher QoS to RELIABLE
                    '-p', 'qos_overrides./points.publisher.reliability:=reliable',
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            load_stdout, load_stderr = load_proc.communicate(timeout=10)
            load_info = load_stdout.decode() + load_stderr.decode()

            # Give the loaded component time to initialize
            time.sleep(2.0)

            # Create synthetic messages
            width = 4
            height = 4
            frame_id = 'test_camera_frame'

            # Depth image: 16UC1, all pixels at 1000mm = 1.0m
            depth_msg = Image()
            depth_msg.header.frame_id = frame_id
            depth_msg.header.stamp = node.get_clock().now().to_msg()
            depth_msg.height = height
            depth_msg.width = width
            depth_msg.encoding = '16UC1'
            depth_msg.is_bigendian = False
            depth_msg.step = width * 2
            depth_data = np.ones((height, width), dtype=np.uint16) * 1000
            depth_msg.data = depth_data.tobytes()

            # RGB image: RGB8, all red
            rgb_msg = Image()
            rgb_msg.header.frame_id = frame_id
            rgb_msg.header.stamp = depth_msg.header.stamp
            rgb_msg.height = height
            rgb_msg.width = width
            rgb_msg.encoding = 'rgb8'
            rgb_msg.is_bigendian = False
            rgb_msg.step = width * 3
            rgb_data = np.zeros((height, width, 3), dtype=np.uint8)
            rgb_data[:, :, 0] = 255  # Red channel
            rgb_msg.data = rgb_data.tobytes()

            # Camera info with simple pinhole model
            info_msg = CameraInfo()
            info_msg.header.frame_id = frame_id
            info_msg.header.stamp = depth_msg.header.stamp
            info_msg.height = height
            info_msg.width = width
            info_msg.distortion_model = 'plumb_bob'
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            fx = 2.0
            fy = 2.0
            cx = 2.0
            cy = 2.0
            info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

            # Also subscribe with RELIABLE in case the node publishes RELIABLE
            cloud_sub_reliable = node.create_subscription(
                PointCloud2,
                '/points',
                cloud_cb,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                )
            )

            # Publish multiple times to ensure synchronization
            timeout = time.time() + 12.0
            while time.time() < timeout and len(received_clouds) == 0:
                stamp = node.get_clock().now().to_msg()
                depth_msg.header.stamp = stamp
                rgb_msg.header.stamp = stamp
                info_msg.header.stamp = stamp

                depth_pub.publish(depth_msg)
                rgb_pub.publish(rgb_msg)
                info_pub.publish(info_msg)

                # Spin to process callbacks
                end_spin = time.time() + 0.3
                while time.time() < end_spin:
                    rclpy.spin_once(node, timeout_sec=0.05)

                time.sleep(0.1)

            # Verify we got a cloud
            assert len(received_clouds) > 0, \
                f"No PointCloud2 messages received from depth_image_proc node. Load info: {load_info}"

            cloud = received_clouds[0]
            # Verify basic structure
            assert cloud.header.frame_id == frame_id, \
                f"Expected frame_id '{frame_id}', got '{cloud.header.frame_id}'"
            assert cloud.height == height, \
                f"Expected height {height}, got {cloud.height}"
            assert cloud.width == width, \
                f"Expected width {width}, got {cloud.width}"
            assert cloud.is_dense is False

            # Verify fields exist
            field_names = [f.name for f in cloud.fields]
            assert 'x' in field_names, "Missing 'x' field"
            assert 'y' in field_names, "Missing 'y' field"
            assert 'z' in field_names, "Missing 'z' field"
            assert 'rgb' in field_names, "Missing 'rgb' field"

            # Check that data is non-empty
            assert len(cloud.data) > 0, "PointCloud2 data is empty"

        finally:
            if proc is not None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()