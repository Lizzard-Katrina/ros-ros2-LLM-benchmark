"""
Runtime test for task_010_husky_stress_test.

This test:
1. Verifies the translated files (husky.gazebo.xacro, husky_empty_world.launch) exist
   and have correct content (interface-level checks).
2. Actually runs the sensor_publisher_node and verifies real ROS2 messages
   are published on the expected topics with correct data.
"""

import pytest
import subprocess
import time
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped


# Locate the package files relative to this test
PACKAGE_ROOT = Path(__file__).resolve().parent
XACRO_FILE = PACKAGE_ROOT / "husky.gazebo.xacro"
LAUNCH_FILE = PACKAGE_ROOT / "husky_empty_world.launch"


class TestFileExistenceAndContent:
    """Test group: verify translated files exist and have correct content."""

    def test_xacro_file_exists(self):
        assert XACRO_FILE.exists(), f"Missing: {XACRO_FILE}"

    def test_launch_file_exists(self):
        assert LAUNCH_FILE.exists(), f"Missing: {LAUNCH_FILE}"

    def test_xacro_has_robot_link_joint(self):
        content = XACRO_FILE.read_text()
        assert "robot" in content
        assert "link" in content
        assert "joint" in content

    def test_xacro_has_imu_interface(self):
        content = XACRO_FILE.read_text()
        imu_keywords = ["imu", "inertial", "sensor", "imu_controller"]
        assert any(kw in content for kw in imu_keywords), "IMU interface missing"

    def test_xacro_has_gps_interface(self):
        content = XACRO_FILE.read_text()
        gps_keywords = ["gps", "navsat", "fix", "gps_controller"]
        assert any(kw in content for kw in gps_keywords), "GPS interface missing"

    def test_xacro_has_update_rate(self):
        content = XACRO_FILE.read_text()
        rate_keywords = ["updateRate", "frequency", "hz", "publish_rate"]
        assert any(kw in content for kw in rate_keywords), "Update rate missing"

    def test_launch_has_robot_description(self):
        content = LAUNCH_FILE.read_text()
        assert "robot_description" in content

    def test_launch_no_ros1_artifacts(self):
        content = LAUNCH_FILE.read_text()
        forbidden = ["<node pkg=", "$(find", "rostopic", "rosparam", "launch"]
        for pat in forbidden:
            assert pat not in content, f"ROS1 artifact found: {pat}"


class TestSensorPublisherRuntime:
    """Test group: actually run the sensor_publisher_node and verify messages."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        """Start the sensor publisher node as a subprocess, then clean up."""
        rclpy.init()
        self.proc = None
        try:
            # Run the node using the module directly
            self.proc = subprocess.Popen(
                [
                    sys.executable, '-m',
                    'task_010_husky_stress_test.sensor_publisher_node'
                ],
                cwd=str(PACKAGE_ROOT),
                env={**os.environ, 'PYTHONPATH': str(PACKAGE_ROOT) + ':' + os.environ.get('PYTHONPATH', '')},
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            # Give the node time to start
            time.sleep(2.0)
            yield
        finally:
            if self.proc is not None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            rclpy.shutdown()

    def test_imu_messages_received(self):
        """Verify IMU messages are published on /imu/data with correct frame_id."""
        node = rclpy.create_node('test_imu_subscriber')
        received_msgs = []

        def imu_callback(msg):
            received_msgs.append(msg)

        sub = node.create_subscription(Imu, 'imu/data', imu_callback, 10)

        timeout = time.time() + 5.0
        try:
            while time.time() < timeout and len(received_msgs) < 3:
                rclpy.spin_once(node, timeout_sec=0.1)

            assert len(received_msgs) >= 3, \
                f"Expected at least 3 IMU messages, got {len(received_msgs)}"

            msg = received_msgs[0]
            assert msg.header.frame_id == 'imu_link', \
                f"Expected frame_id 'imu_link', got '{msg.header.frame_id}'"
            # Check gravity on z-axis
            assert abs(msg.linear_acceleration.z - 9.81) < 0.1, \
                f"Expected ~9.81 m/s^2 on z, got {msg.linear_acceleration.z}"
            # Check orientation is valid quaternion
            assert abs(msg.orientation.w - 1.0) < 0.01
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()

    def test_gps_messages_received(self):
        """Verify GPS NavSatFix messages are published on /navsat/fix."""
        node = rclpy.create_node('test_gps_subscriber')
        received_msgs = []

        def gps_callback(msg):
            received_msgs.append(msg)

        sub = node.create_subscription(NavSatFix, 'navsat/fix', gps_callback, 10)

        timeout = time.time() + 5.0
        try:
            while time.time() < timeout and len(received_msgs) < 2:
                rclpy.spin_once(node, timeout_sec=0.1)

            assert len(received_msgs) >= 2, \
                f"Expected at least 2 GPS messages, got {len(received_msgs)}"

            msg = received_msgs[0]
            assert msg.header.frame_id == 'navsat_link', \
                f"Expected frame_id 'navsat_link', got '{msg.header.frame_id}'"
            # Check reference coordinates from xacro
            assert abs(msg.latitude - 49.9) < 0.1, \
                f"Expected latitude ~49.9, got {msg.latitude}"
            assert abs(msg.longitude - 8.9) < 0.1, \
                f"Expected longitude ~8.9, got {msg.longitude}"
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()

    def test_gps_vel_messages_received(self):
        """Verify GPS velocity messages are published on /navsat/vel."""
        node = rclpy.create_node('test_gps_vel_subscriber')
        received_msgs = []

        def vel_callback(msg):
            received_msgs.append(msg)

        sub = node.create_subscription(TwistStamped, 'navsat/vel', vel_callback, 10)

        timeout = time.time() + 5.0
        try:
            while time.time() < timeout and len(received_msgs) < 2:
                rclpy.spin_once(node, timeout_sec=0.1)

            assert len(received_msgs) >= 2, \
                f"Expected at least 2 GPS vel messages, got {len(received_msgs)}"

            msg = received_msgs[0]
            assert msg.header.frame_id == 'navsat_link', \
                f"Expected frame_id 'navsat_link', got '{msg.header.frame_id}'"
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()