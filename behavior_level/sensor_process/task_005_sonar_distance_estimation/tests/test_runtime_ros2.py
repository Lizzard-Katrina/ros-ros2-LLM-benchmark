"""
Runtime test for task_005_sonar_distance_estimation.

Tests the kinematics and sonar logic by:
1. Running the kinematics_node (which implements the same math as turtle.cpp)
2. Sending velocity commands and verifying pose updates and sonar readings
3. Also verifying turtle.cpp source passes oracle checks
"""
import subprocess
import sys
import time
import math
import os
import re
from pathlib import Path

import pytest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


@pytest.fixture(scope='module')
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def find_turtle_cpp():
    """Find turtle.cpp in the package."""
    # Check package root first
    candidates = [
        Path(__file__).resolve().parent / "turtle.cpp",
        Path(__file__).resolve().parent / "src" / "turtle.cpp",
    ]
    # Also check installed share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('task_005_sonar_distance_estimation')
        candidates.append(Path(share_dir) / "turtle.cpp")
    except Exception:
        pass

    for c in candidates:
        if c.exists():
            return c
    return None


class TestStaticOracle:
    """Verify turtle.cpp passes the static oracle checks."""

    def get_content(self):
        cpp_file = find_turtle_cpp()
        assert cpp_file is not None, "Could not find turtle.cpp"
        with open(cpp_file, 'r') as f:
            return re.sub(r'//.*?\n|/\*.*?\*/', '', f.read(), flags=re.DOTALL)

    def test_holonomic_kinematics(self):
        content = self.get_content()
        x_pattern = r"pos_\.rx\(\)\s*\+?=\s*.*?(?:cos|sin).*?lin_vel_x_"
        y_pattern = r"pos_\.ry\(\)\s*[-+]=\s*.*?(?:cos|sin).*?lin_vel_y_"
        assert re.search(x_pattern, content) and re.search(y_pattern, content)

    def test_sonar_geometry(self):
        content = self.get_content()
        intersection_math = r"/\s*(?:dx|dy|(?:std::)?(?:cos|sin)\()"
        assert re.search(intersection_math, content) and "canvas_width" in content

    def test_numerical_stability_epsilon(self):
        content = self.get_content()
        stability_check = r"(?:std::)?(?:abs|fabs)\(d[xy]\)\s*>\s*(?:0|1e-|0\.)"
        assert re.search(stability_check, content)

    def test_sonar_max_range_limit(self):
        content = self.get_content()
        range_limit = r"(?:sonar_distance_|sonar_dist).*?=\s*.*?(?:\d+\.\d+|range_max|max_range)"
        assert re.search(range_limit, content)

    def test_sonar_y_mirroring(self):
        content = self.get_content()
        assert re.search(r"dy\s*=\s*-\s*(?:std::)?sin", content)

    def test_frame_transformation(self):
        content = self.get_content()
        y_flip = r"p->y\s*=\s*canvas_height\s*-\s*pos_\.y\(\)"
        assert re.search(y_flip, content)


class TestRuntimeKinematics:
    """Runtime test: launch the Python kinematics node and verify behavior."""

    def test_pose_update_and_sonar(self, ros_init):
        """
        Launch kinematics_node, send a velocity command, and verify
        that pose updates correctly and sonar returns a valid distance.
        """
        node_proc = None
        test_node = None
        try:
            # Launch the kinematics node
            node_proc = subprocess.Popen(
                [sys.executable, '-m', 'scripts.kinematics_node'],
                cwd=str(Path(__file__).resolve().parent),
                env={**os.environ, 'PYTHONPATH': str(Path(__file__).resolve().parent) + ':' + os.environ.get('PYTHONPATH', '')},
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Give it time to start
            time.sleep(2.0)

            test_node = rclpy.create_node('test_kinematics_verifier')

            # Storage for received messages
            received_poses = []
            received_sonar = []

            def pose_cb(msg):
                received_poses.append(msg.data)

            def sonar_cb(msg):
                received_sonar.append(msg.data)

            pose_sub = test_node.create_subscription(
                Float64MultiArray, 'turtle_pose', pose_cb, 10)
            sonar_sub = test_node.create_subscription(
                Float64MultiArray, 'sonar_distance', sonar_cb, 10)
            cmd_pub = test_node.create_publisher(Twist, 'cmd_vel', 10)

            # Wait for initial pose messages
            timeout = time.time() + 5.0
            while time.time() < timeout and len(received_poses) < 3:
                rclpy.spin_once(test_node, timeout_sec=0.1)

            assert len(received_poses) > 0, "No pose messages received from kinematics_node"

            # Check initial pose (should be near center: 5.5, 5.5 in internal, published as 5.5, 5.5)
            initial_pose = received_poses[-1]
            assert len(initial_pose) == 3, "Pose should have 3 elements (x, y, theta)"
            assert abs(initial_pose[0] - 5.5) < 0.5, f"Initial x should be ~5.5, got {initial_pose[0]}"
            assert abs(initial_pose[1] - 5.5) < 0.5, f"Initial y should be ~5.5, got {initial_pose[1]}"

            # Check initial sonar (from center, should be ~5.5 or capped at max_range=5.0)
            assert len(received_sonar) > 0, "No sonar messages received"
            initial_sonar = received_sonar[-1][0]
            assert 0.0 < initial_sonar <= 5.5, f"Initial sonar should be in (0, 5.5], got {initial_sonar}"

            # Now send a forward velocity command
            received_poses.clear()
            received_sonar.clear()

            cmd = Twist()
            cmd.linear.x = 2.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            # Publish several times to ensure it's received
            for _ in range(10):
                cmd_pub.publish(cmd)
                rclpy.spin_once(test_node, timeout_sec=0.05)

            # Wait for pose updates
            time.sleep(1.0)
            timeout = time.time() + 3.0
            while time.time() < timeout and len(received_poses) < 10:
                rclpy.spin_once(test_node, timeout_sec=0.1)

            assert len(received_poses) > 5, "Not enough pose updates after velocity command"

            # The turtle should have moved in the +x direction (orient=0 means cos(0)=1)
            # Internal pos_x increases, published x increases
            last_pose = received_poses[-1]
            assert last_pose[0] > 5.5, f"Turtle should have moved right, x={last_pose[0]}"

            # Sonar should still be valid and finite
            assert len(received_sonar) > 0, "No sonar after movement"
            last_sonar = received_sonar[-1][0]
            assert 0.0 < last_sonar <= 5.0, f"Sonar should be in (0, 5.0], got {last_sonar}"

            # As turtle moves right, sonar distance to right wall should decrease
            # (since orient=0, sonar looks right)
            assert last_sonar < initial_sonar or abs(last_sonar - initial_sonar) < 0.5, \
                "Sonar should decrease as turtle approaches right wall"

        finally:
            if test_node:
                test_node.destroy_node()
            if node_proc:
                node_proc.terminate()
                try:
                    node_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    node_proc.kill()

    def test_kinematics_math_directly(self, ros_init):
        """
        Import and test the kinematics math functions directly from the
        kinematics_node module.
        """
        # Import the actual module
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from scripts.kinematics_node import update_pose, compute_sonar_distance, normalize_angle

        # Test normalize_angle
        assert abs(normalize_angle(0.0)) < 1e-9
        assert abs(normalize_angle(2 * math.pi)) < 1e-6
        assert abs(normalize_angle(-math.pi) - (-math.pi)) < 1e-6

        # Test basic forward motion (orient=0, moving right in internal coords)
        canvas_w, canvas_h = 11.0, 11.0
        x, y, theta, sonar, hit = update_pose(
            5.5, 5.5, 0.0,  # pos_x, pos_y, orient
            1.0, 0.0, 0.0,  # lin_vel_x, lin_vel_y, ang_vel
            1.0,             # dt
            canvas_w, canvas_h)

        # With orient=0, cos(0)=1, so x should increase by 1.0
        assert abs(x - 6.5) < 1e-6, f"Expected x=6.5, got {x}"
        # y should stay the same (sin(0)=0)
        assert abs(y - 5.5) < 1e-6, f"Expected y=5.5, got {y}"
        assert not hit

        # Test sonar from center facing right (orient=0)
        sonar_dist = compute_sonar_distance(5.5, 5.5, 0.0, canvas_w, canvas_h)
        # Facing right, distance to right wall (x=11) is 5.5
        assert sonar_dist <= 5.0, f"Sonar should be capped at max_range=5.0, got {sonar_dist}"
        assert sonar_dist > 0.0

        # Test sonar from near a wall
        sonar_dist = compute_sonar_distance(10.0, 5.5, 0.0, canvas_w, canvas_h)
        # Facing right, distance to right wall is 1.0
        assert abs(sonar_dist - 1.0) < 0.5, f"Expected sonar ~1.0, got {sonar_dist}"

        # Test boundary collision
        x, y, theta, sonar, hit = update_pose(
            10.5, 5.5, 0.0,
            5.0, 0.0, 0.0,
            1.0,
            canvas_w, canvas_h)
        assert hit, "Should have hit the wall"
        assert x == canvas_w, f"x should be clamped to {canvas_w}, got {x}"

        # Test holonomic motion (orient=pi/2, moving in y)
        x, y, theta, sonar, hit = update_pose(
            5.5, 5.5, math.pi / 2.0,
            1.0, 0.0, 0.0,
            1.0,
            canvas_w, canvas_h)
        # orient=pi/2: cos(pi/2)~0, -sin(pi/2)=-1
        # x += cos(pi/2)*1.0 ~ 0, y += -sin(pi/2)*1.0 = -1.0
        assert abs(x - 5.5) < 0.1, f"Expected x~5.5, got {x}"
        assert abs(y - 4.5) < 0.1, f"Expected y~4.5, got {y}"

        # Test Y-axis flip in published pose
        # Published y = canvas_height - pos_y
        published_y = canvas_h - y
        assert abs(published_y - 6.5) < 0.1, f"Published y should be ~6.5, got {published_y}"

        # Test numerical stability: orient along axis (no crash)
        sonar_dist = compute_sonar_distance(5.5, 5.5, 0.0, canvas_w, canvas_h)
        assert math.isfinite(sonar_dist)
        sonar_dist = compute_sonar_distance(5.5, 5.5, math.pi / 2.0, canvas_w, canvas_h)
        assert math.isfinite(sonar_dist)
        sonar_dist = compute_sonar_distance(5.5, 5.5, math.pi, canvas_w, canvas_h)
        assert math.isfinite(sonar_dist)