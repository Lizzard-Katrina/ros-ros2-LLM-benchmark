"""
Runtime test for the diff_drive_controller ROS 2 node.
Launches the node as a subprocess and verifies:
1. The node starts and declares expected parameters with correct defaults.
2. The odom topic is advertised.
3. Parameters can be read back with expected default values.
"""

import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_diff_drive_node_parameters(ros_context):
    """
    Launch the diff_drive_controller node and verify parameters are declared
    with correct default values.
    """
    proc = None
    test_node = None
    try:
        # Launch the node as a subprocess
        proc = subprocess.Popen(
            ["ros2", "run", "task_003_Mobile_robot_control", "diff_drive_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the node time to start up
        time.sleep(3.0)

        # Check the process is still running
        assert proc.poll() is None, (
            f"Node exited prematurely with code {proc.returncode}. "
            f"stderr: {proc.stderr.read().decode() if proc.stderr else 'N/A'}"
        )

        # Create a test node to query parameters
        test_node = Node("test_param_checker")

        # Use ros2 param CLI to get parameter values
        # Check cmd_vel_timeout default = 0.5
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "cmd_vel_timeout"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get cmd_vel_timeout: {result.stderr}"
        assert "0.5" in result.stdout, f"Expected default 0.5, got: {result.stdout}"

        # Check open_loop default = False
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "open_loop"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get open_loop: {result.stderr}"
        assert "False" in result.stdout, f"Expected default False, got: {result.stdout}"

        # Check velocity_rolling_window_size default = 10
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "velocity_rolling_window_size"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get velocity_rolling_window_size: {result.stderr}"
        assert "10" in result.stdout, f"Expected default 10, got: {result.stdout}"

        # Check publish_rate default = 50.0
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "publish_rate"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get publish_rate: {result.stderr}"
        assert "50.0" in result.stdout, f"Expected default 50.0, got: {result.stdout}"

        # Check base_frame_id default = "base_link"
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "base_frame_id"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get base_frame_id: {result.stderr}"
        assert "base_link" in result.stdout, f"Expected default base_link, got: {result.stdout}"

        # Check pose_covariance_diagonal is declared (vector parameter)
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "pose_covariance_diagonal"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get pose_covariance_diagonal: {result.stderr}"
        # Should contain some of the default values
        assert "0.001" in result.stdout, f"Expected 0.001 in pose_covariance_diagonal, got: {result.stdout}"

        # Check twist_covariance_diagonal is declared
        result = subprocess.run(
            ["ros2", "param", "get", "/diff_drive_controller", "twist_covariance_diagonal"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to get twist_covariance_diagonal: {result.stderr}"
        assert "0.001" in result.stdout, f"Expected 0.001 in twist_covariance_diagonal, got: {result.stdout}"

        # Verify the odom topic exists
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, f"Failed to list topics: {result.stderr}"
        assert "/diff_drive_controller/odom" in result.stdout or "/odom" in result.stdout, \
            f"Expected odom topic, got: {result.stdout}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)