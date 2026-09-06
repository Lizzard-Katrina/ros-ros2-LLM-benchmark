"""
Runtime test for task_009_fetch_arm_params.

This test verifies the migrated C++ source file contains the correct ROS2 patterns
by reading and analyzing the actual translated file, and also creates a minimal
ROS2 node to verify parameter declaration patterns work at runtime.
"""
import os
import re
import time
import subprocess
import pytest
import rclpy
from rclpy.node import Node
from pathlib import Path


# Find the source file - it could be in the package root or installed share dir
def find_source_file():
    """Locate the translated C++ source file."""
    # Check package root first
    candidates = [
        Path(__file__).parent / "fetch_arm_ikfast_moveit_plugin.cpp",
    ]
    # Also check installed share directory
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "task_009_fetch_arm_params"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            prefix = result.stdout.strip()
            candidates.append(
                Path(prefix) / "share" / "task_009_fetch_arm_params" / "fetch_arm_ikfast_moveit_plugin.cpp"
            )
    except Exception:
        pass

    for c in candidates:
        if c.exists():
            return c
    return None


@pytest.fixture(scope="module")
def source_code():
    """Load the translated source file content."""
    src = find_source_file()
    assert src is not None, "Could not find fetch_arm_ikfast_moveit_plugin.cpp"
    return src.read_text()


def test_ros2_param_declaration(source_code):
    """Verify ROS2 'Declare before Get' requirement."""
    pattern = r"node_->declare_parameter\s*<\s*(?:std::string|double|int)\s*>\s*\(\s*\"[\w_]+\""
    assert re.search(pattern, source_code), \
        "Missing mandatory ROS 2 parameter declarations via node_->declare_parameter."


def test_snake_case_naming(source_code):
    """Verify parameters follow ROS2 snake_case convention."""
    assert "robot_description" in source_code, \
        "Parameter 'robot_description' should be snake_case."
    assert "robotDescription" not in source_code, \
        "Detected legacy ROS 1 CamelCase parameter naming."


def test_logging_migration(source_code):
    """Verify transition from ROS_ERROR/INFO to node-based RCLCPP macros."""
    assert re.search(r"RCLCPP_(?:ERROR|INFO|DEBUG|WARN)", source_code), \
        "No ROS 2 logging macros (RCLCPP_*) found."
    # The initialize function should not have ROS_ERROR - check that the TODO section uses RCLCPP
    # Note: we check the whole file doesn't have bare ROS_ERROR (not in comments)
    # Filter out lines that are comments
    non_comment_lines = []
    for line in source_code.split('\n'):
        stripped = line.strip()
        if not stripped.startswith('//') and not stripped.startswith('*'):
            non_comment_lines.append(line)
    non_comment_code = '\n'.join(non_comment_lines)
    assert "ROS_ERROR" not in non_comment_code, \
        "Legacy ROS 1 logging (ROS_ERROR) should be removed."
    assert "get_logger()" in source_code, \
        "RCLCPP macros must use node_->get_logger()."


def test_frame_consistency(source_code):
    """Verify the solver references hardcoded frames."""
    assert "IKFAST_BASE_FRAME_" in source_code, \
        "Initialization must account for IKFAST_BASE_FRAME_."
    assert "IKFAST_TIP_FRAME_" in source_code, \
        "Initialization must account for IKFAST_TIP_FRAME_."


def test_moveit2_api_usage(source_code):
    """Verify usage of MoveIt 2 RobotModel/JointModelGroup API."""
    assert "getJointModelGroup" in source_code, \
        "Missing call to getJointModelGroup for validation."
    assert "getVariableBounds" in source_code or "getjointlimits" in source_code.lower(), \
        "Should retrieve joint limits from RobotModel."


def test_tip_frame_validation(source_code):
    """Verify semantic check for IKFast 6DOF constraint (1 tip frame)."""
    assert re.search(r"tip_frames\.size\(\)\s*(!=|==|>)\s*1", source_code), \
        "Should validate that exactly one tip frame is provided."


def test_initialize_function_present(source_code):
    """Verify the initialize function is fully implemented (not just a TODO)."""
    # Check that initialize has actual implementation, not just TODO
    # Find the initialize function body
    init_match = re.search(
        r"IKFastKinematicsPlugin::initialize\s*\([^)]*\)\s*\{(.*?)^\}",
        source_code, re.DOTALL | re.MULTILINE
    )
    assert init_match is not None, "Could not find initialize function implementation."
    body = init_match.group(1)
    # Should have substantial code, not just TODO comments
    # Count non-comment, non-empty lines
    code_lines = [
        l.strip() for l in body.split('\n')
        if l.strip() and not l.strip().startswith('//') and not l.strip().startswith('*')
    ]
    assert len(code_lines) > 10, \
        f"Initialize function body too short ({len(code_lines)} lines), likely not implemented."


def test_node_parameter_declaration_runtime():
    """
    Runtime test: Create a real ROS2 node and verify that the parameter
    declaration pattern used in the translated code actually works.
    This exercises the ROS2 parameter API that the plugin relies on.
    """
    rclpy.init()
    try:
        node = Node("test_param_declaration_node")

        # Mimic the parameter declarations from the translated initialize() function
        node.declare_parameter("robot_description", "")
        node.declare_parameter("kinematics_solver_search_resolution", 0.005)
        node.declare_parameter("kinematics_solver_attempts", 3)
        node.declare_parameter("kinematics_solver_timeout", 5.0)

        # Verify we can get the parameters back
        robot_desc = node.get_parameter("robot_description").get_parameter_value().string_value
        assert robot_desc == "", f"Expected empty string, got '{robot_desc}'"

        search_res = node.get_parameter("kinematics_solver_search_resolution").get_parameter_value().double_value
        assert abs(search_res - 0.005) < 1e-9, f"Expected 0.005, got {search_res}"

        attempts = node.get_parameter("kinematics_solver_attempts").get_parameter_value().integer_value
        assert attempts == 3, f"Expected 3, got {attempts}"

        timeout_val = node.get_parameter("kinematics_solver_timeout").get_parameter_value().double_value
        assert abs(timeout_val - 5.0) < 1e-9, f"Expected 5.0, got {timeout_val}"

        # Verify that getting an undeclared parameter raises (ROS2 behavior)
        with pytest.raises(Exception):
            node.get_parameter("undeclared_param_xyz")

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_no_ros1_includes(source_code):
    """Verify ROS1 headers are replaced with ROS2 equivalents."""
    # Should not have ros/ros.h
    assert "#include <ros/ros.h>" not in source_code, \
        "Should not include ROS1 header ros/ros.h"
    # Should have rclcpp
    assert "rclcpp" in source_code, \
        "Should include rclcpp header"


def test_msg_namespace_migration(source_code):
    """Verify message types use ROS2 namespacing (msg::)."""
    # In ROS2, it's geometry_msgs::msg::Pose not geometry_msgs::Pose
    # Check that the initialize function and surrounding code uses ::msg::
    assert "geometry_msgs::msg::Pose" in source_code or "moveit_msgs::msg::" in source_code, \
        "Should use ROS2 message namespacing (::msg::)"