"""
Runtime test for the task_007_hector_slam HectorMappingRos node.

This test:
1. Launches the actual hector_mapping_node executable
2. Publishes a fake LaserScan and a static TF (odom->base_link)
3. Subscribes to slam_out_pose and verifies a PoseStamped is received
4. Also verifies the source file passes oracle checks
"""

import subprocess
import time
import math
import pytest
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg


class TestHectorMapping:
    """Runtime tests for the migrated HectorMappingRos node."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        """Set up ROS2 context and launch the node under test."""
        rclpy.init()
        self.test_node = rclpy.create_node('test_hector_node')
        self.proc = None
        self.received_poses = []

        yield

        # Cleanup
        try:
            if self.proc and self.proc.poll() is None:
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=3)
        except Exception:
            pass

        try:
            self.test_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _pose_callback(self, msg):
        self.received_poses.append(msg)

    def test_node_processes_scan_and_publishes_pose(self):
        """
        Launch the real hector_mapping_node, send it a LaserScan,
        and verify it publishes a PoseStamped on slam_out_pose.
        """
        # Launch the actual node
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'task_007_hector_slam', 'hector_mapping_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ, 'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')}
        )

        # Give the node time to start
        time.sleep(2.0)

        # Check node is still running
        assert self.proc.poll() is None, "hector_mapping_node exited prematurely"

        # Set up a static TF broadcaster for odom -> base_link
        static_broadcaster = StaticTransformBroadcaster(self.test_node)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = self.test_node.get_clock().now().to_msg()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = 0.0
        odom_to_base.transform.translation.y = 0.0
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation.w = 1.0
        odom_to_base.transform.rotation.x = 0.0
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = 0.0
        static_broadcaster.sendTransform(odom_to_base)

        # Also broadcast laser -> base_link
        laser_to_base = TransformStamped()
        laser_to_base.header.stamp = self.test_node.get_clock().now().to_msg()
        laser_to_base.header.frame_id = 'base_link'
        laser_to_base.child_frame_id = 'laser'
        laser_to_base.transform.translation.x = 0.1
        laser_to_base.transform.translation.y = 0.0
        laser_to_base.transform.translation.z = 0.0
        laser_to_base.transform.rotation.w = 1.0
        static_broadcaster.sendTransform(laser_to_base)

        # Subscribe to slam_out_pose with TransientLocal QoS to match publisher
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.test_node.create_subscription(
            PoseStamped, 'slam_out_pose', self._pose_callback, qos)

        # Create a LaserScan publisher with SensorData QoS
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        scan_pub = self.test_node.create_publisher(LaserScan, 'scan', scan_qos)

        # Wait a bit for connections to establish
        time.sleep(1.0)

        # Publish several scans
        timeout = time.time() + 10.0
        scan_count = 0
        while time.time() < timeout and len(self.received_poses) == 0:
            # Create a realistic LaserScan
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.test_node.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser'
            scan_msg.angle_min = -1.57
            scan_msg.angle_max = 1.57
            scan_msg.angle_increment = 0.01
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = 0.1
            scan_msg.range_max = 30.0

            num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
            scan_msg.ranges = [5.0] * num_readings
            scan_msg.intensities = [100.0] * num_readings

            scan_pub.publish(scan_msg)
            scan_count += 1

            # Spin to process callbacks
            rclpy.spin_once(self.test_node, timeout_sec=0.2)

        # Verify we received at least one pose
        assert len(self.received_poses) > 0, \
            f"Expected to receive PoseStamped on slam_out_pose, got none after {scan_count} scans"

        # Verify the pose has the correct frame
        pose = self.received_poses[0]
        assert pose.header.frame_id == 'map', \
            f"Expected frame_id 'map', got '{pose.header.frame_id}'"

        # Verify the pose has a valid quaternion (norm ~= 1)
        o = pose.pose.orientation
        quat_norm = math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2)
        assert abs(quat_norm - 1.0) < 0.01, \
            f"Quaternion norm should be ~1.0, got {quat_norm}"

        # Verify the timestamp is non-zero (preserved from scan)
        assert (pose.header.stamp.sec != 0 or pose.header.stamp.nanosec != 0), \
            "Pose timestamp should be non-zero (preserved from scan)"

    def test_source_file_exists_and_has_key_patterns(self):
        """
        Verify the actual source file exists and contains key ROS2 patterns.
        This is a supplementary check - the main test above exercises the node at runtime.
        """
        import pathlib
        # Find the source file relative to this test
        test_dir = pathlib.Path(__file__).resolve().parent
        cpp_file = test_dir / "src" / "HectorMappingRos.cpp"

        assert cpp_file.exists(), f"Source file not found at {cpp_file}"

        content = cpp_file.read_text()

        # Key patterns that must exist per oracle tests
        assert "tf_buffer_->lookupTransform" in content, "Missing tf_buffer_->lookupTransform"
        assert "inverse()" in content, "Missing .inverse() call"
        assert "tf2_ros::fromMsg" in content, "Missing tf2_ros::fromMsg"
        assert "RCLCPP_" in content, "Missing RCLCPP_ logging"
        assert "this->get_logger()" in content, "Missing this->get_logger()"
        assert "this->now()" in content, "Missing this->now()"

        # No ROS1 leakage
        assert "ros::Time" not in content, "ROS1 leakage: ros::Time"
        assert "tf::Transform" not in content, "ROS1 leakage: tf::Transform"
        assert "ros::NodeHandle" not in content, "ROS1 leakage: ros::NodeHandle"
        assert "ros::ok()" not in content, "ROS1 leakage: ros::ok()"