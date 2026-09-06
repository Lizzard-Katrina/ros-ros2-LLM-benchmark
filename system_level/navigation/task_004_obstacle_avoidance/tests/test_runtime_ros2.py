"""
Runtime test for task_004_obstacle_avoidance.

Tests:
1. The draw_square node publishes on turtle1/cmd_vel when it receives pose data.
2. The source files contain the correct ROS2 patterns (no ROS1 remnants).
"""

import os
import re
import time
import signal
import subprocess
import pytest
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Structural / source-level checks ──────────────────────────────────────

BASE_PATH = Path(__file__).resolve().parent

def _read(name):
    p = BASE_PATH / name
    return p.read_text() if p.exists() else ""


def test_draw_square_has_state_machine():
    """draw_square.cpp must define an FSM with FORWARD state."""
    content = _read("draw_square.cpp")
    assert re.search(r"enum\s+\w+\s*{\s*FORWARD", content, re.S), \
        "draw_square.cpp missing enum with FORWARD state"


def test_draw_square_uses_pi_half():
    """draw_square.cpp must use PI/2 for 90-degree turns."""
    content = _read("draw_square.cpp")
    assert re.search(r"(?:PI\s*/\s*2|1\.57|M_PI_2)", content), \
        "draw_square.cpp missing PI/2 turn logic"


def test_draw_square_uses_current_pose():
    """draw_square.cpp must use current_pose_ for closed-loop feedback."""
    content = _read("draw_square.cpp")
    assert re.search(r"current_pose_\.(?:x|y|theta)", content), \
        "draw_square.cpp missing current_pose_ feedback"


def test_turtle_frame_declares_parameters():
    """turtle_frame.cpp must declare background parameters with descriptors."""
    content = _read("turtle_frame.cpp")
    assert re.search(r'declare_parameter\s*\(\s*"background_[rgb]"', content), \
        "turtle_frame.cpp missing declare_parameter for background colors"
    assert re.search(r"ParameterDescriptor|IntegerRange", content), \
        "turtle_frame.cpp missing ParameterDescriptor or IntegerRange"


def test_turtle_frame_parameter_events():
    """turtle_frame.cpp must subscribe to parameter_events and call update()."""
    content = _read("turtle_frame.cpp")
    assert "parameter_events" in content, \
        "turtle_frame.cpp missing parameter_events subscription"
    assert "update()" in content, \
        "turtle_frame.cpp missing update() call in parameter event handler"


def test_cmake_has_required_elements():
    """CMakeLists.txt must have draw_square executable and turtlesim dependency."""
    content = _read("CMakeLists.txt")
    assert "add_executable(draw_square" in content, \
        "CMakeLists.txt missing add_executable(draw_square"
    assert "turtlesim" in content, \
        "CMakeLists.txt missing turtlesim"


def test_package_xml_has_deps():
    """package.xml must declare all required dependencies."""
    content = _read("package.xml")
    for dep in ["rclcpp", "geometry_msgs", "turtlesim"]:
        assert re.search(rf"<(?:depend|build_depend|exec_depend)>{dep}", content, re.I), \
            f"package.xml missing dependency: {dep}"


def test_no_ros1_api_in_sources():
    """No ROS1 API patterns should appear in any source file."""
    all_content = _read("draw_square.cpp") + _read("turtle_frame.cpp") + _read("CMakeLists.txt")
    for pattern in [r"ros::init", r"ros::NodeHandle", r"ros::Publisher", r"catkin"]:
        assert not re.search(pattern, all_content, re.I), \
            f"ROS1 remnant detected: {pattern}"


# ── Live ROS2 interaction test ─────────────────────────────────────────────

class TwistCollector(Node):
    """Helper node that subscribes to turtle1/cmd_vel and collects messages."""

    def __init__(self):
        super().__init__("twist_collector_test")
        self.received = []
        self.sub = self.create_subscription(
            Twist, "turtle1/cmd_vel", self._cb, 10
        )

    def _cb(self, msg):
        self.received.append(msg)


def test_draw_square_publishes_cmd_vel():
    """
    Launch the draw_square node and feed it a fake pose. Verify it publishes
    Twist messages on turtle1/cmd_vel.
    """
    rclpy.init()
    collector = None
    proc = None
    pose_pub_node = None
    try:
        collector = TwistCollector()

        # Launch the draw_square node as a subprocess
        proc = subprocess.Popen(
            ["ros2", "run", "task_004_obstacle_avoidance", "draw_square"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the node a moment to start
        time.sleep(1.5)

        # Create a node that publishes fake pose data (simulating turtlesim)
        pose_pub_node = rclpy.create_node("fake_pose_publisher")

        # Import turtlesim Pose (ROS2 Humble: turtlesim.msg.Pose)
        from turtlesim.msg import Pose

        pose_pub = pose_pub_node.create_publisher(Pose, "turtle1/pose", 10)

        # Publish pose messages to trigger the draw_square state machine
        pose_msg = Pose()
        pose_msg.x = 5.0
        pose_msg.y = 5.0
        pose_msg.theta = 0.0
        pose_msg.linear_velocity = 0.0
        pose_msg.angular_velocity = 0.0

        deadline = time.time() + 8.0
        while time.time() < deadline and len(collector.received) < 3:
            pose_pub.publish(pose_msg)
            rclpy.spin_once(collector, timeout_sec=0.05)
            rclpy.spin_once(pose_pub_node, timeout_sec=0.02)

        assert len(collector.received) >= 1, \
            f"Expected at least one Twist message on turtle1/cmd_vel, got {len(collector.received)}"

        # The draw_square node should publish forward motion (linear.x=1.0)
        # when it first receives a pose and sets up the FORWARD goal
        found_forward = any(
            abs(msg.linear.x - 1.0) < 0.01 for msg in collector.received
        )
        assert found_forward, \
            "Expected at least one Twist with linear.x ~1.0 (FORWARD state)"

    finally:
        if proc is not None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if collector:
            collector.destroy_node()
        if pose_pub_node:
            pose_pub_node.destroy_node()
        rclpy.try_shutdown()