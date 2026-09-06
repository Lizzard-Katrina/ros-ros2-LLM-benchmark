"""
Runtime test for task_008_navigation_recovery_action.

Tests that:
1. The translated C++ source file exists and passes static oracle checks.
2. The compiled executable can be launched.
3. The node creates the expected subscriptions and service client.
4. The node calls /gazebo/set_model_state with the expected arguments.
"""

import subprocess
import time
import os
import re
import sys
import json
import signal
import tempfile
import pytest


# Path to the C++ source file (at package root, same level as this test)
CPP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gazebo_model_states.cpp")
MOCK_SERVICE_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_test_mock_service_node.py")


def get_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        return f.read()


def test_source_file_exists():
    """The translated C++ file must exist."""
    assert os.path.isfile(CPP_FILE), f"Source file not found at {CPP_FILE}"


def test_no_ros1_artifacts():
    """No ROS1 headers or namespace calls remain."""
    content = get_content()
    assert not re.search(r"#include <ros/ros\.h>", content), "Legacy ROS1 header found!"
    assert not re.search(r"\bros::", content), "Legacy ros:: namespace usage found!"
    assert not re.search(r"gazebo_msgs/ModelStates\.h", content), "Legacy message header format found!"


def test_ros2_subscriptions_in_source():
    """Verify subscriptions to /gazebo/model_states and /gazebo/link_states exist."""
    content = get_content()
    assert re.search(
        r'create_subscription<gazebo_msgs::msg::ModelStates>\s*\(\s*"/gazebo/model_states"', content
    ), "Missing or incorrect ROS2 subscription to /gazebo/model_states"
    assert re.search(
        r'create_subscription<gazebo_msgs::msg::LinkStates>\s*\(\s*"/gazebo/link_states"', content
    ), "Missing or incorrect ROS2 subscription to /gazebo/link_states"


def test_ros2_service_client_in_source():
    """Verify service client for /gazebo/set_model_state is created and invoked."""
    content = get_content()
    assert re.search(
        r'create_client<gazebo_msgs::srv::SetModelState>\s*\(\s*"/gazebo/set_model_state"', content
    ), "Service client for /gazebo/set_model_state was not created using ROS2 API."
    assert re.search(r"set_model_state\s*\(.*?\)", content), \
        "The set_model_state helper function is not called."


def test_pose_and_twist_initialization():
    """Ensure Pose and Twist objects are initialized and passed to the service."""
    content = get_content()
    assert re.search(r"geometry_msgs::msg::Pose", content), "geometry_msgs::msg::Pose not found."
    assert re.search(r"geometry_msgs::msg::Twist", content), "geometry_msgs::msg::Twist not found."
    assert re.search(
        r'set_model_state\s*\(\s*"ball"\s*,\s*"world"\s*,\s*.*?\s*,\s*.*?\)', content
    ), "Service call does not pass required Pose and Twist arguments."


def test_runtime_node_interaction():
    """
    Launch the compiled node and a mock service, verify the service was called
    with the expected arguments by reading from a temp file written by the mock.
    """
    # Create a temp file for the mock service to write results to
    tmp_fd, tmp_path = tempfile.mkstemp(suffix='.json', prefix='mock_srv_')
    os.close(tmp_fd)
    # Remove it so we can detect when it's written
    os.unlink(tmp_path)

    mock_proc = None
    node_proc = None

    try:
        # Start the mock service node first
        mock_proc = subprocess.Popen(
            [sys.executable, MOCK_SERVICE_SCRIPT, tmp_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for the mock service to be registered
        time.sleep(3.0)

        # Launch the compiled executable
        node_proc = subprocess.Popen(
            ["ros2", "run", "task_008_navigation_recovery_action", "gazebo_model_states"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for the node to call the service and the mock to write the file
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if os.path.isfile(tmp_path):
                # Give a tiny bit more time for file write to complete
                time.sleep(0.2)
                break
            time.sleep(0.2)

        # Check that the node process completed (it should exit after service call)
        if node_proc.poll() is None:
            node_proc.send_signal(signal.SIGINT)
            node_proc.wait(timeout=5)

        # Verify the file was created (service was called)
        assert os.path.isfile(tmp_path), (
            "The /gazebo/set_model_state service was never called by the node."
        )

        with open(tmp_path, 'r') as f:
            data = json.load(f)

        assert data['model_name'] == 'ball', (
            f"Expected model_name='ball', got '{data['model_name']}'"
        )
        assert data['reference_frame'] == 'world', (
            f"Expected reference_frame='world', got '{data['reference_frame']}'"
        )
        assert abs(data['pose']['position']['z'] - 1.0) < 1e-6, (
            f"Expected pose.position.z=1.0, got {data['pose']['position']['z']}"
        )
        assert abs(data['twist']['linear']['x']) < 1e-6, "twist.linear.x should be 0"
        assert abs(data['twist']['linear']['y']) < 1e-6, "twist.linear.y should be 0"
        assert abs(data['twist']['linear']['z']) < 1e-6, "twist.linear.z should be 0"

    finally:
        if node_proc is not None:
            try:
                if node_proc.poll() is None:
                    node_proc.send_signal(signal.SIGINT)
                    node_proc.wait(timeout=5)
            except Exception:
                node_proc.kill()
                node_proc.wait(timeout=5)
        if mock_proc is not None:
            try:
                mock_proc.send_signal(signal.SIGINT)
                mock_proc.wait(timeout=5)
            except Exception:
                mock_proc.kill()
                mock_proc.wait(timeout=5)
        # Clean up temp file
        if os.path.isfile(tmp_path):
            os.unlink(tmp_path)