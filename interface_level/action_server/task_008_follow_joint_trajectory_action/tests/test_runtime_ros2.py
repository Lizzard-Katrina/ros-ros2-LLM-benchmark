"""
Runtime test for task_008_follow_joint_trajectory_action.

This test validates the translated trajectory_planner_ros.cpp file
by parsing its content and verifying the ROS2 translation is correct
at runtime. It also spins up a minimal ROS2 node to confirm the
ROS2 environment is functional and the message types referenced in
the translated code actually exist.
"""

import re
import time
import subprocess
import threading
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Locate the translated source file
# ---------------------------------------------------------------------------
# The file lives at the package root (same directory level or installed share)
_THIS_DIR = Path(__file__).resolve().parent
_CPP_CANDIDATES = [
    _THIS_DIR / "trajectory_planner_ros.cpp",
    # If installed via colcon
    Path("/ros_ws/install/task_008_follow_joint_trajectory_action/share/"
         "task_008_follow_joint_trajectory_action/trajectory_planner_ros.cpp"),
]

CPP_FILE = None
for p in _CPP_CANDIDATES:
    if p.exists():
        CPP_FILE = p
        break


def _read_code():
    assert CPP_FILE is not None and CPP_FILE.exists(), \
        f"Cannot find trajectory_planner_ros.cpp in any candidate path"
    code = CPP_FILE.read_text()
    # Strip block comments and line comments for cleaner matching
    code_no_block = re.sub(r'/\*[\s\S]*?\*/', '', code)
    code_clean = re.sub(r'//.*', '', code_no_block)
    return code_clean


# ---------------------------------------------------------------------------
# Test 1: Verify the class and method structure
# ---------------------------------------------------------------------------
def test_class_and_method_exist():
    code = _read_code()
    assert re.search(r"class\s+TrajectoryPlannerROS", code), \
        "TrajectoryPlannerROS class not found"
    assert re.search(r"bool\s+.*checkTrajectory\s*\(", code), \
        "checkTrajectory method not found"


# ---------------------------------------------------------------------------
# Test 2: Verify costmap_ros_->getRobotPose is used
# ---------------------------------------------------------------------------
def test_get_robot_pose():
    code = _read_code()
    assert re.search(r"costmap_ros_->getRobotPose", code), \
        "costmap_ros_->getRobotPose not found"


# ---------------------------------------------------------------------------
# Test 3: Verify update_map logic
# ---------------------------------------------------------------------------
def test_update_map_logic():
    code = _read_code()
    assert re.search(r"if\s*\(\s*update_map\s*\)", code), \
        "update_map conditional not found"
    assert re.search(r"tc_->updatePlan", code), \
        "tc_->updatePlan not found in update_map branch"


# ---------------------------------------------------------------------------
# Test 4: Verify odom lock usage
# ---------------------------------------------------------------------------
def test_odom_lock():
    code = _read_code()
    assert re.search(r"boost::recursive_mutex::scoped_lock", code), \
        "boost::recursive_mutex::scoped_lock not found"
    assert re.search(r"base_odom\s*=\s*base_odom_", code), \
        "Odometry copy inside lock not found"


# ---------------------------------------------------------------------------
# Test 5: Verify tc_->checkTrajectory call with correct arguments
# ---------------------------------------------------------------------------
def test_tc_check_trajectory_call():
    code = _read_code()
    pattern = (
        r"tc_->checkTrajectory\s*\(\s*global_pose\.pose\.position\.x\s*,"
        r"\s*global_pose\.pose\.position\.y\s*,"
        r"\s*tf2::getYaw\(global_pose\.pose\.orientation\)\s*,"
        r"\s*base_odom\.twist\.twist\.linear\.x\s*,"
        r"\s*base_odom\.twist\.twist\.linear\.y\s*,"
        r"\s*base_odom\.twist\.twist\.angular\.z\s*,"
        r"\s*vx_samp\s*,\s*vy_samp\s*,\s*vtheta_samp"
    )
    assert re.search(pattern, code, re.DOTALL), \
        "tc_->checkTrajectory call with correct args not found"


# ---------------------------------------------------------------------------
# Test 6: Verify return of boolean from tc_->checkTrajectory
# ---------------------------------------------------------------------------
def test_returns_bool():
    code = _read_code()
    assert re.search(r"return\s+tc_->checkTrajectory", code), \
        "checkTrajectory must return boolean result of tc_->checkTrajectory"


# ---------------------------------------------------------------------------
# Test 7: Verify warning on getRobotPose failure
# ---------------------------------------------------------------------------
def test_warn_on_pose_failure():
    code = CPP_FILE.read_text()  # Use raw code (with comments) for WARN check
    assert re.search(r"(RCLCPP_WARN|ROS_WARN).*Failed to get the pose", code), \
        "Missing warning for failed getRobotPose"


# ---------------------------------------------------------------------------
# Test 8: No ROS1 API leftovers
# ---------------------------------------------------------------------------
def test_no_ros1_leftovers():
    code = _read_code()
    forbidden = [r"ros::", r"tf::", r"nav_msgs::Odometry", r"costmap_2d::Costmap2DROS"]
    for f in forbidden:
        assert not re.search(f, code), f"ROS1 API leftover detected: {f}"


# ---------------------------------------------------------------------------
# Test 9: Uses ROS2 message types
# ---------------------------------------------------------------------------
def test_uses_ros2_message_types():
    code = CPP_FILE.read_text()
    # Should use ROS2-style message namespacing
    assert re.search(r"geometry_msgs::msg::PoseStamped", code), \
        "Should use geometry_msgs::msg::PoseStamped (ROS2 style)"
    assert re.search(r"nav_msgs::msg::Odometry", code), \
        "Should use nav_msgs::msg::Odometry (ROS2 style)"


# ---------------------------------------------------------------------------
# Test 10: Actually exercise ROS2 runtime – spin a node and verify
#          that the message types referenced in the code are importable
#          and usable at runtime.
# ---------------------------------------------------------------------------
def test_ros2_runtime_message_types():
    """Spin a real ROS2 node, create the message types used in the
    translated code, and verify they work."""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = None
    try:
        node = rclpy.create_node('test_task008_runtime')

        # Create a PoseStamped (as used in checkTrajectory)
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 2.0

        # Create an Odometry message (as used in checkTrajectory)
        odom = Odometry()
        odom.twist.twist.linear.x = 0.5
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.1
        assert odom.twist.twist.linear.x == 0.5
        assert odom.twist.twist.angular.z == 0.1

        # Verify the node is alive
        assert node.get_name() == 'test_task008_runtime'

        # Create a publisher for Odometry (as the real code would subscribe to)
        pub = node.create_publisher(Odometry, '/odom', 10)
        assert pub is not None

        # Publish and verify no crash
        pub.publish(odom)

        # Spin once to process
        rclpy.spin_once(node, timeout_sec=0.5)

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Test 11: Verify the translated code uses RCLCPP_WARN (ROS2 logging)
# ---------------------------------------------------------------------------
def test_uses_rclcpp_logging():
    code = CPP_FILE.read_text()
    assert re.search(r"RCLCPP_WARN", code), \
        "Should use RCLCPP_WARN for ROS2 logging"


# ---------------------------------------------------------------------------
# Test 12: Verify rclcpp include
# ---------------------------------------------------------------------------
def test_includes_rclcpp():
    code = CPP_FILE.read_text()
    assert re.search(r"#include\s*<rclcpp/rclcpp\.hpp>", code), \
        "Should include <rclcpp/rclcpp.hpp>"