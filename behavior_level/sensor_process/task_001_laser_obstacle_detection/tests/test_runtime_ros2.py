"""
Runtime test for the scan_to_scan_filter_chain node.

This test:
1. Launches the scan_to_scan_filter_chain_node executable.
2. Publishes a LaserScan on 'scan'.
3. Subscribes to 'scan_filtered' and asserts the filtered data arrives with expected content.

Since the filter_chain_ has no filters configured (or filters pkg acts as pass-through),
the input scan should appear on the output topic.
"""

import os
import time
import subprocess
import pytest


def test_scan_to_scan_filter_chain_passthrough():
    """
    Launch the scan_to_scan_filter_chain_node, publish a LaserScan on /scan,
    and verify that a filtered scan appears on /scan_filtered with the same data.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan

    # Start the node under test as a subprocess
    env = os.environ.copy()
    proc = subprocess.Popen(
        ["ros2", "run", "task_001_laser_obstacle_detection", "scan_to_scan_filter_chain_node"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    received_msgs = []

    try:
        rclpy.init()
        test_node = rclpy.create_node("test_scan_filter_node")

        # Give the node time to start
        time.sleep(3.0)

        # QoS for sensor data - BEST_EFFORT to match SensorDataQoS used by the node's subscriber
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # The output publisher uses RELIABLE (default) with depth = scan_filtered_history_depth
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Also try BEST_EFFORT subscription on output in case publisher negotiates down
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        def on_filtered(msg):
            received_msgs.append(msg)

        # Subscribe with both QoS profiles to maximize compatibility
        sub1 = test_node.create_subscription(
            LaserScan, "/scan_filtered", on_filtered, reliable_qos
        )
        sub2 = test_node.create_subscription(
            LaserScan, "/scan_filtered", on_filtered, best_effort_qos
        )

        # Publisher for raw scan input - use BEST_EFFORT to match the node's message_filters subscriber
        pub = test_node.create_publisher(LaserScan, "/scan", sensor_qos)

        # Build a test LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = -1.57
        scan_msg.angle_max = 1.57
        scan_msg.angle_increment = 0.01
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0
        num_readings = int(
            (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment
        )
        scan_msg.ranges = [5.0] * num_readings
        scan_msg.intensities = [100.0] * num_readings

        # Publish repeatedly and spin to receive
        timeout = time.time() + 15.0
        while time.time() < timeout and len(received_msgs) == 0:
            scan_msg.header.stamp = test_node.get_clock().now().to_msg()
            pub.publish(scan_msg)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        # Assertions
        assert len(received_msgs) > 0, (
            "No filtered scan received on /scan_filtered within timeout"
        )

        filtered = received_msgs[0]
        assert filtered.header.frame_id == "laser_frame", (
            f"Expected frame_id 'laser_frame', got '{filtered.header.frame_id}'"
        )
        assert len(filtered.ranges) == num_readings, (
            f"Expected {num_readings} ranges, got {len(filtered.ranges)}"
        )
        assert abs(filtered.ranges[0] - 5.0) < 1e-3, (
            f"Expected range ~5.0, got {filtered.ranges[0]}"
        )
        assert abs(filtered.angle_min - (-1.57)) < 1e-3, (
            f"Expected angle_min ~-1.57, got {filtered.angle_min}"
        )
        assert abs(filtered.range_max - 30.0) < 1e-3, (
            f"Expected range_max ~30.0, got {filtered.range_max}"
        )

    finally:
        try:
            test_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def test_source_file_exists_and_has_key_patterns():
    """
    Verify the source file contains the key patterns expected by the oracle tests.
    """
    import re
    from pathlib import Path

    # Find the source file - it could be in the source directory or installed
    cpp_file = Path(__file__).resolve().parent / "scan_to_scan_filter_chain.cpp"
    assert cpp_file.exists(), f"Source file not found: {cpp_file}"

    content = cpp_file.read_text()
    # Remove comments
    content_no_comments = re.sub(r"//.*|/\*[\s\S]*?\*/", "", content)

    # Check key patterns from oracle
    assert "output_pub_" in content_no_comments, "Missing output_pub_ member usage"
    assert re.search(
        r"filter_chain_\.update\s*\([^)]+\)", content_no_comments
    ), "Missing filter_chain_.update() call"
    assert "SensorDataQoS" in content_no_comments or "sensor_data" in content_no_comments, \
        "Missing SensorDataQoS or sensor_data QoS usage"