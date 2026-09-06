"""
Runtime test for task_005_laser_scan_filter_params.

This test launches the generic_laser_filter_node, publishes a LaserScan
message on the 'scan' topic, and verifies the node forwards it to 'output'.
"""

import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def node_process():
    """Launch the generic_laser_filter_node as a subprocess."""
    proc = subprocess.Popen(
        ["ros2", "run", "task_005_laser_scan_filter_params", "generic_laser_filter_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    # Give the node time to start
    time.sleep(3.0)
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


def test_node_is_running(ros_context, node_process):
    """Verify the node process started and hasn't crashed."""
    assert node_process.poll() is None, (
        f"Node process terminated unexpectedly with code {node_process.returncode}"
    )


def test_node_name_in_graph(ros_context, node_process):
    """Verify the node name 'scan_filter_node' appears in the ROS 2 graph."""
    test_node = Node("test_graph_checker")
    try:
        deadline = time.time() + 8.0
        found = False
        while time.time() < deadline:
            node_names = [n[0] for n in test_node.get_node_names_and_namespaces()]
            if "scan_filter_node" in node_names:
                found = True
                break
            rclpy.spin_once(test_node, timeout_sec=0.5)
        assert found, (
            f"'scan_filter_node' not found in graph. Found: {node_names}"
        )
    finally:
        test_node.destroy_node()


def test_output_topic_exists(ros_context, node_process):
    """Verify the 'output' topic is advertised by the node."""
    test_node = Node("test_topic_checker")
    try:
        deadline = time.time() + 8.0
        found = False
        while time.time() < deadline:
            topic_names = [t[0] for t in test_node.get_topic_names_and_types()]
            if "/output" in topic_names:
                found = True
                break
            rclpy.spin_once(test_node, timeout_sec=0.5)
        assert found, (
            f"'/output' topic not found. Found topics: {topic_names}"
        )
    finally:
        test_node.destroy_node()


def test_scan_topic_subscription(ros_context, node_process):
    """Verify the node subscribes to 'scan' topic."""
    test_node = Node("test_scan_checker")
    try:
        deadline = time.time() + 8.0
        found = False
        while time.time() < deadline:
            topic_names = [t[0] for t in test_node.get_topic_names_and_types()]
            if "/scan" in topic_names:
                found = True
                break
            rclpy.spin_once(test_node, timeout_sec=0.5)
        assert found, (
            f"'/scan' topic not found. Found topics: {topic_names}"
        )
    finally:
        test_node.destroy_node()


def test_publish_and_verify_output(ros_context, node_process):
    """
    Publish a LaserScan on /scan and check that the node publishes on /output
    with the expected data (passthrough mode when no filters configured).
    """
    assert node_process.poll() is None, (
        f"Node crashed before publish test with code {node_process.returncode}"
    )

    test_node = Node("test_pub_sub")
    received_msgs = []

    def output_cb(msg):
        received_msgs.append(msg)

    try:
        pub = test_node.create_publisher(LaserScan, "/scan", 10)
        sub = test_node.create_subscription(
            LaserScan, "/output", output_cb, 10
        )

        # Build a scan message
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = -1.0
        scan_msg.angle_max = 1.0
        scan_msg.angle_increment = 0.1
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [1.0] * 20
        scan_msg.intensities = [100.0] * 20

        deadline = time.time() + 8.0
        while time.time() < deadline and len(received_msgs) == 0:
            scan_msg.header.stamp = test_node.get_clock().now().to_msg()
            pub.publish(scan_msg)
            rclpy.spin_once(test_node, timeout_sec=0.1)

        # Verify we received at least one message on /output
        assert len(received_msgs) > 0, "No messages received on /output topic"

        # Verify the content matches what we sent (passthrough mode)
        out = received_msgs[0]
        assert out.header.frame_id == "base_link", (
            f"Expected frame_id 'base_link', got '{out.header.frame_id}'"
        )
        assert abs(out.angle_min - (-1.0)) < 1e-5, (
            f"Expected angle_min -1.0, got {out.angle_min}"
        )
        assert abs(out.angle_max - 1.0) < 1e-5, (
            f"Expected angle_max 1.0, got {out.angle_max}"
        )
        assert len(out.ranges) == 20, (
            f"Expected 20 ranges, got {len(out.ranges)}"
        )
        assert abs(out.ranges[0] - 1.0) < 1e-5, (
            f"Expected range value 1.0, got {out.ranges[0]}"
        )
        assert abs(out.range_max - 10.0) < 1e-5, (
            f"Expected range_max 10.0, got {out.range_max}"
        )

        # Node should still be alive
        assert node_process.poll() is None, "Node crashed during publish test"

    finally:
        test_node.destroy_node()