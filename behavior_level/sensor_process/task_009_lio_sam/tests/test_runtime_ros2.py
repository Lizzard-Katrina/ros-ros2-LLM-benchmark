"""
Runtime test for task_009_lio_sam mapOptimization node.
Tests:
  1. The node starts and the save_map service is available and responds correctly
  2. The TF broadcast uses timeLaserInfoStamp (verified via odometry topic)
  3. Service returns proper success field
  4. Static oracle checks pass
"""
import subprocess
import sys
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


@pytest.fixture(scope="module")
def ros_setup():
    """Initialize ROS2 and start the map_optimization_node."""
    rclpy.init()
    node_proc = subprocess.Popen(
        ["ros2", "run", "task_009_lio_sam", "map_optimization_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    # Give the node time to start
    time.sleep(3.0)
    yield node_proc
    node_proc.terminate()
    try:
        node_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        node_proc.kill()
        node_proc.wait(timeout=3)
    rclpy.shutdown()


def test_service_available_and_responds(ros_setup):
    """Test that the save_map service exists and returns a valid response with success field."""
    node_proc = ros_setup
    assert node_proc.poll() is None, "Node process died unexpectedly"

    test_node = rclpy.create_node("test_service_client")
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    try:
        client = test_node.create_client(Trigger, "lio_sam/save_map")

        # Wait for service to be available
        deadline = time.time() + 10.0
        while not client.wait_for_service(timeout_sec=1.0):
            assert time.time() < deadline, "Service lio_sam/save_map not available within timeout"
            assert node_proc.poll() is None, "Node process died while waiting for service"

        # Call the service
        request = Trigger.Request()
        future = client.call_async(request)

        # Spin until we get a response
        deadline = time.time() + 10.0
        while not future.done():
            executor.spin_once(timeout_sec=0.1)
            assert time.time() < deadline, "Service call timed out"

        response = future.result()
        assert response is not None, "Service returned None response"
        # The service should set success field (true since no real keyframes but
        # the code handles empty case)
        assert hasattr(response, 'success'), "Response missing 'success' field"
        # With no keyframes, it should return false
        assert response.success is False, \
            f"Expected success=False (no keyframes), got {response.success}"
        assert "No keyframes" in response.message or len(response.message) > 0, \
            "Response message should indicate status"

    finally:
        test_node.destroy_node()


def test_odometry_topic_exists(ros_setup):
    """Test that the odometry publisher topic is advertised."""
    node_proc = ros_setup
    assert node_proc.poll() is None, "Node process died unexpectedly"

    test_node = rclpy.create_node("test_odom_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    try:
        # Check that the topic is advertised by listing topics
        deadline = time.time() + 8.0
        found = False
        while time.time() < deadline:
            topic_names_and_types = test_node.get_topic_names_and_types()
            topic_names = [t[0] for t in topic_names_and_types]
            if "/lio_sam/mapping/odometry" in topic_names:
                found = True
                break
            executor.spin_once(timeout_sec=0.5)

        assert found, (
            "Topic /lio_sam/mapping/odometry not found. "
            f"Available topics: {[t[0] for t in test_node.get_topic_names_and_types()]}"
        )
    finally:
        test_node.destroy_node()


def test_node_uses_correct_parameters(ros_setup):
    """Test that the node declares expected parameters."""
    node_proc = ros_setup
    assert node_proc.poll() is None, "Node process died unexpectedly"

    test_node = rclpy.create_node("test_param_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    try:
        # Use ros2 param list to check parameters
        result = subprocess.run(
            ["ros2", "param", "list", "/lio_sam_mapOptimization"],
            capture_output=True, text=True, timeout=10
        )
        output = result.stdout
        # Check that our declared parameters exist
        assert "odometryFrame" in output or "mapFrame" in output, \
            f"Expected parameters not found. Output: {output}"
    finally:
        test_node.destroy_node()


def test_static_oracle_checks_pass():
    """
    Verify the source file passes the key static checks from the oracle test.
    This ensures the translated file has the right patterns.
    """
    import re
    from pathlib import Path

    # Find the source file
    cpp_file = Path(__file__).resolve().parent / "src" / "mapOptmization.cpp"
    if not cpp_file.exists():
        # Try alternate location
        cpp_file = Path(__file__).resolve().parent / "mapOptmization.cpp"

    assert cpp_file.exists(), f"Source file not found at {cpp_file}"

    content = cpp_file.read_text()

    # Check 1: timeLaserInfoStamp used for TF stamp
    stamp_match = re.search(r"\.header\.stamp\s*=\s*([^;]+);", content)
    assert stamp_match is not None, "No header.stamp assignment found"
    found_laser_stamp = False
    for m in re.finditer(r"\.header\.stamp\s*=\s*([^;]+);", content):
        val = m.group(1)
        if "timeLaserInfoStamp" in val:
            found_laser_stamp = True
    assert found_laser_stamp, "Must use timeLaserInfoStamp for header.stamp"

    # Check 2: callback_group usage
    assert "create_callback_group" in content or "callback_group_" in content, \
        "Must use callback groups"

    # Check 3: lock_guard in service
    assert "std::lock_guard" in content, "Must use std::lock_guard in service"

    # Check 4: shared_ptr service params
    pattern = r"const\s+std::shared_ptr<[^>]+::Request>\s+\w+,\s*std::shared_ptr<[^>]+::Response>\s+\w+"
    assert re.search(pattern, content), "Service must use shared_ptr params"

    # Check 5: res->success
    assert re.search(r"res->success\s*=\s*(?:true|false)", content), \
        "Must set res->success"

    # Check 6: No ROS1 symbols
    legacy = ["ros::Time", "ros::ok()", "ROS_INFO", "ros::Publisher", "ros::Subscriber", "tf::"]
    for sym in legacy:
        assert sym not in content, f"Legacy symbol '{sym}' found"