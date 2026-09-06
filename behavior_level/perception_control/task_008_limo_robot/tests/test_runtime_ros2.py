"""
Runtime test for the task_008_limo_robot package.
Tests the translated limo_driver.cpp by:
1. Verifying the source file passes the oracle regex checks.
2. Verifying the package builds and the executable exists.
3. Running a live ROS2 test: launching the node, checking topics.
"""

import re
import os
import time
import subprocess
import signal
import pytest
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ── Locate the source file ──
PKG_ROOT = Path(__file__).resolve().parent
CPP_FILE = PKG_ROOT / "limo_driver.cpp"
SRC_CPP_FILE = PKG_ROOT / "src" / "limo_driver.cpp"


def _read_source():
    """Read the limo_driver.cpp source."""
    for p in [CPP_FILE, SRC_CPP_FILE]:
        if p.exists():
            return p.read_text()
    return ""


def _get_func_body(source, func_name):
    """Extract the full body of a C++ member function, handling nested braces."""
    # Find the function signature
    pattern = rf'LimoDriver::{func_name}\s*\([^)]*\)\s*\{{'
    match = re.search(pattern, source)
    if not match:
        # Try without class qualifier
        pattern = rf'{func_name}\s*\([^)]*\)\s*\{{'
        match = re.search(pattern, source)
    if not match:
        return ""

    # Now count braces to find the matching closing brace
    start = match.end() - 1  # position of the opening {
    depth = 0
    i = start
    while i < len(source):
        if source[i] == '{':
            depth += 1
        elif source[i] == '}':
            depth -= 1
            if depth == 0:
                return source[start + 1:i]
        i += 1
    return source[start + 1:]


class TestOracleChecks:
    """Re-run the static oracle checks to make sure the source is correct."""

    @classmethod
    def setup_class(cls):
        cls.source = _read_source()
        assert cls.source, "limo_driver.cpp not found"

    def test_ackermann_inverse_kinematics(self):
        blk = _get_func_body(self.source, "twistCmdCallback")
        # Check for atan(wheelbase_ / r) pattern
        has_math = re.search(r"atan\(wheelbase_.*?/.*?r\)", blk) or \
                   re.search(r"wheelbase_.*?/.*?tan", blk)
        assert has_math, "Missing Ackermann geometry (atan/tan) using wheelbase."

    def test_steering_limit_clamping(self):
        blk = _get_func_body(self.source, "twistCmdCallback")
        has_clamping = "max_inner_angle_" in blk and ("if" in blk or "clamp" in blk)
        assert has_clamping, "Steering angle must be clamped by max_inner_angle_."

    def test_odom_integration_frames(self):
        blk = _get_func_body(self.source, "publishOdometry")
        # Check for position_x_ += ... cos(theta_) ... * dt  pattern
        has_rot_x = re.search(r"position_x_.*?(\+=|=).*?cos\(.*?theta", blk)
        has_rot_y = re.search(r"position_y_.*?(\+=|=).*?sin\(.*?theta", blk)
        assert has_rot_x and has_rot_y, "Incorrect velocity projection into global frame."

    def test_mecanum_lateral_awareness(self):
        blk = _get_func_body(self.source, "publishOdometry")
        is_aware = "lateral_velocity" in blk and "vy" in blk
        assert is_aware, "Lateral velocity (vy) ignored in Mecanum odometry."

    def test_time_differential_consistency(self):
        blk = _get_func_body(self.source, "publishOdometry")
        assert "dt" in blk and re.search(r"\*\s*dt", blk), \
            "Pose update missing time delta (dt) scaling."

    def test_protocol_bit_shifting(self):
        blk = _get_func_body(self.source, "setMotionCommand")
        has_bits = ">> 8" in blk and "& 0x" in blk.lower()
        assert has_bits, "Failed to serialize 16-bit commands into byte-stream."

    def test_ackermann_tan_in_odom(self):
        blk = _get_func_body(self.source, "publishOdometry")
        has_tan = "tan" in blk and "wheelbase_" in blk
        assert has_tan, "Ackermann odom must use tan(steering_angle)/wheelbase_."

    def test_normalize_angle(self):
        blk = _get_func_body(self.source, "normalizeAngle")
        assert "M_PI" in blk or "3.14" in blk, "normalizeAngle must reference pi."


class TestBuildAndExecutable:
    """Verify the package compiled and the executable exists."""

    def test_executable_exists(self):
        result = subprocess.run(
            ["ros2", "pkg", "executables", "task_008_limo_robot"],
            capture_output=True, text=True, timeout=10
        )
        assert "limo_base_node" in result.stdout, \
            f"limo_base_node executable not found. Output: {result.stdout} {result.stderr}"


class TestRuntimeOdom:
    """
    Launch the limo_base_node with port_name=none (so it skips serial),
    and verify the node starts and advertises expected topics.
    """

    def test_node_starts_and_topics_exist(self):
        rclpy.init()
        proc = None
        test_node = None
        try:
            env = os.environ.copy()
            proc = subprocess.Popen(
                ["ros2", "run", "task_008_limo_robot", "limo_base_node",
                 "--ros-args", "-p", "port_name:=none"],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            time.sleep(2.0)

            assert proc.poll() is None, \
                f"Node exited prematurely with code {proc.returncode}"

            test_node = rclpy.create_node("test_limo_checker")

            deadline = time.time() + 5.0
            odom_found = False
            cmd_vel_found = False

            while time.time() < deadline:
                topic_names = [t[0] for t in test_node.get_topic_names_and_types()]
                if "/odom" in topic_names:
                    odom_found = True
                if "/cmd_vel" in topic_names:
                    cmd_vel_found = True
                if odom_found and cmd_vel_found:
                    break
                rclpy.spin_once(test_node, timeout_sec=0.2)

            assert odom_found, f"Topic /odom not found. Topics: {topic_names}"
            assert cmd_vel_found, f"Topic /cmd_vel not found. Topics: {topic_names}"

            # Publish a cmd_vel message to exercise the callback
            pub = test_node.create_publisher(Twist, "/cmd_vel", 10)
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.1
            for _ in range(5):
                pub.publish(twist)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)

            assert proc.poll() is None, "Node crashed after receiving cmd_vel"

        finally:
            if test_node is not None:
                test_node.destroy_node()
            if proc is not None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
            rclpy.shutdown()