#!/usr/bin/env python3
"""
Runtime test for task_004_laser_scan_origin.
Launches the lidar_publisher node via subprocess, then uses a test node
to subscribe to /scan and verify that LaserScan messages arrive with
correct field values.
"""
import subprocess
import sys
import time
import math
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class ScanListener(Node):
    def __init__(self):
        super().__init__('test_scan_listener')
        self.received_msgs = []
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg):
        self.received_msgs.append(msg)


def test_publisher_publishes_laser_scan():
    """Launch the real lidar_publisher and verify LaserScan messages on /scan."""
    proc = subprocess.Popen(
        [sys.executable, '-u', '-m', 'task_004_laser_scan_origin.lidar_publisher'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    listener = ScanListener()
    try:
        timeout = 8.0
        start = time.time()
        while time.time() - start < timeout and len(listener.received_msgs) < 3:
            rclpy.spin_once(listener, timeout_sec=0.1)

        assert len(listener.received_msgs) >= 1, \
            f"Expected at least 1 LaserScan message, got {len(listener.received_msgs)}"

        msg = listener.received_msgs[0]

        # Verify key fields
        assert msg.header.frame_id == 'laser_frame', \
            f"Expected frame_id 'laser_frame', got '{msg.header.frame_id}'"
        assert abs(msg.angle_min - (-math.pi)) < 0.01, \
            f"Expected angle_min ~ -pi, got {msg.angle_min}"
        assert abs(msg.angle_max - math.pi) < 0.01, \
            f"Expected angle_max ~ pi, got {msg.angle_max}"
        assert len(msg.ranges) > 0, "Expected non-empty ranges"
        assert abs(msg.ranges[0] - 1.0) < 0.01, \
            f"Expected ranges[0] ~ 1.0, got {msg.ranges[0]}"
        assert msg.range_min > 0.0, "Expected range_min > 0"
        assert msg.range_max > msg.range_min, "Expected range_max > range_min"

    finally:
        listener.destroy_node()
        proc.terminate()
        proc.wait(timeout=5)


def test_subscriber_receives_and_processes():
    """
    Launch the real lidar_subscriber, publish a LaserScan to /scan from
    the test, and verify the subscriber processes it (by checking stdout).
    """
    proc = subprocess.Popen(
        [sys.executable, '-u', '-m', 'task_004_laser_scan_origin.lidar_subscriber'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    pub_node = Node('test_scan_publisher')
    publisher = pub_node.create_publisher(LaserScan, '/scan', 10)

    try:
        # Give subscriber time to start
        time.sleep(2.0)

        scan = LaserScan()
        scan.header.frame_id = 'test_frame'
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [2.5, 1.2, 0.8, 3.0]
        scan.intensities = [100.0, 100.0, 100.0, 100.0]

        # Publish several times to ensure delivery
        for _ in range(20):
            publisher.publish(scan)
            rclpy.spin_once(pub_node, timeout_sec=0.05)
            time.sleep(0.05)

        # Give subscriber time to process
        time.sleep(1.0)

        proc.terminate()
        stdout, stderr = proc.communicate(timeout=5)
        output = stdout.decode('utf-8', errors='replace')

        # The subscriber should have printed the closest range (0.8)
        assert '0.8000' in output, \
            f"Expected subscriber to print closest range 0.8000, got stdout: {output!r}"

    finally:
        pub_node.destroy_node()
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)


def test_subscriber_module_has_callback():
    """Verify the subscriber module exposes a callback function."""
    from task_004_laser_scan_origin.lidar_subscriber import callback
    assert callable(callback)


def test_publisher_node_class_exists():
    """Verify the publisher module has a Node subclass."""
    from task_004_laser_scan_origin.lidar_publisher import LidarPublisherNode
    assert issubclass(LidarPublisherNode, Node)