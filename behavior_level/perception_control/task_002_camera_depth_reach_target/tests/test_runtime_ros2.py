"""
Runtime test for task_002_camera_depth_reach_target.

This test:
1. Verifies the package built and the executable exists.
2. Runs the executable (stub mode when MoveIt planning interface is absent)
   and verifies it publishes CollisionObject messages on the collision_object topic.
3. Validates the source file (depth_reach.cpp) contains all required ROS2 MoveIt2 constructs.
"""

import re
import os
import subprocess
import pathlib
import time
import threading
import pytest

FLAGS = re.MULTILINE | re.DOTALL


def _find_source():
    """Locate depth_reach.cpp in multiple possible locations."""
    candidates = [
        pathlib.Path(__file__).parent / "src" / "depth_reach.cpp",
        pathlib.Path(__file__).parent / "depth_reach.cpp",
    ]
    # Also check installed share directory
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", "task_002_camera_depth_reach_target"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            share_path = pathlib.Path(result.stdout.strip()) / "src" / "depth_reach.cpp"
            candidates.insert(0, share_path)
    except Exception:
        pass

    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError(
        f"depth_reach.cpp not found in any of: {[str(c) for c in candidates]}"
    )


def _read_source():
    return _find_source().read_text(encoding="utf-8", errors="ignore")


def _has(pat, s):
    return re.search(pat, s, FLAGS) is not None


# ---------------------------------------------------------------------------
# Test: executable exists and runs, publishes CollisionObject messages
# ---------------------------------------------------------------------------

def test_executable_runs():
    """Run the depth_reach executable and verify it produces expected output."""
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = None
    proc = None
    received_objects = []

    try:
        node = rclpy.create_node("_test_collision_listener")

        from moveit_msgs.msg import CollisionObject

        def cb(msg):
            if msg.id not in received_objects:
                received_objects.append(msg.id)

        # Match the transient local QoS used by the publisher
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(CollisionObject, "collision_object", cb, qos)

        # Start spinning in a background thread so we're ready before the publisher
        spin_active = True

        def spin_thread():
            while spin_active and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)

        spinner = threading.Thread(target=spin_thread, daemon=True)
        spinner.start()

        # Give the subscriber a moment to be fully set up
        time.sleep(0.5)

        # Launch the executable
        proc = subprocess.Popen(
            ["ros2", "run", "task_002_camera_depth_reach_target", "depth_reach"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for up to 20 seconds for the process to finish and messages to arrive
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if proc.poll() is not None:
                # Process exited; wait a bit more for late messages
                time.sleep(1.0)
                break
            time.sleep(0.2)

        # Stop spinner
        spin_active = False
        spinner.join(timeout=3.0)

        # The stub publishes table1, table2, object
        assert "table1" in received_objects, \
            f"Expected 'table1' in received collision objects, got: {received_objects}"
        assert "table2" in received_objects, \
            f"Expected 'table2' in received collision objects, got: {received_objects}"
        assert "object" in received_objects, \
            f"Expected 'object' in received collision objects, got: {received_objects}"

    except ImportError as e:
        pytest.skip(f"Required message packages not installed: {e}")
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Test: source file uses ROS2 APIs (not ROS1)
# ---------------------------------------------------------------------------

def test_source_uses_rclcpp():
    s = _read_source()
    assert _has(r'#include\s*[<"]\s*rclcpp/rclcpp\.hpp\s*[>"]', s), \
        "Source must include rclcpp/rclcpp.hpp"
    assert not _has(r'#include\s*[<"]\s*ros/ros\.h\s*[>"]', s), \
        "Source must NOT include ros/ros.h"


def test_source_has_rclcpp_init_spin_shutdown():
    s = _read_source()
    assert _has(r'rclcpp::init', s), "Missing rclcpp::init"
    assert _has(r'(rclcpp::spin|executor\.spin|\.spin\(\))', s), "Missing spin"
    assert _has(r'rclcpp::shutdown', s), "Missing rclcpp::shutdown"


# ---------------------------------------------------------------------------
# Test: collision objects
# ---------------------------------------------------------------------------

def test_collision_objects():
    s = _read_source()
    assert _has(r'"table1"', s), "Missing table1 collision object"
    assert _has(r'"table2"', s), "Missing table2 collision object"
    assert _has(r'"object"', s), "Missing 'object' collision object"
    assert _has(r'CollisionObject', s), "Missing CollisionObject type"
    assert _has(r'applyCollisionObjects', s), "Missing applyCollisionObjects call"
    assert _has(r'"panda_link0"', s), "Missing panda_link0 frame_id"


# ---------------------------------------------------------------------------
# Test: pick pipeline
# ---------------------------------------------------------------------------

def test_pick_pipeline():
    s = _read_source()
    assert _has(r'Grasp', s), "Missing Grasp type"
    assert _has(r'pick\s*\(\s*"object"', s), "Missing pick(\"object\") call"
    assert _has(r'grasp_pose', s), "Missing grasp_pose"
    assert _has(r'pre_grasp_approach', s), "Missing pre_grasp_approach"
    assert _has(r'post_grasp_retreat', s), "Missing post_grasp_retreat"
    assert _has(r'pre_grasp_posture', s), "Missing pre_grasp_posture"
    assert _has(r'grasp_posture', s), "Missing grasp_posture"


# ---------------------------------------------------------------------------
# Test: place pipeline
# ---------------------------------------------------------------------------

def test_place_pipeline():
    s = _read_source()
    assert _has(r'PlaceLocation', s), "Missing PlaceLocation type"
    assert _has(r'place\s*\(\s*"object"', s), "Missing place(\"object\") call"
    assert _has(r'place_pose', s), "Missing place_pose"
    assert _has(r'pre_place_approach', s), "Missing pre_place_approach"
    assert _has(r'post_place_retreat', s), "Missing post_place_retreat"
    assert _has(r'post_place_posture', s), "Missing post_place_posture"


# ---------------------------------------------------------------------------
# Test: support surface semantics
# ---------------------------------------------------------------------------

def test_support_surfaces():
    s = _read_source()
    assert _has(r'setSupportSurfaceName\s*\(\s*"table1"\s*\)', s), \
        "Missing setSupportSurfaceName(\"table1\")"
    assert _has(r'setSupportSurfaceName\s*\(\s*"table2"\s*\)', s), \
        "Missing setSupportSurfaceName(\"table2\")"


# ---------------------------------------------------------------------------
# Test: rclpy smoke - verify MoveIt message types are importable
# ---------------------------------------------------------------------------

def test_moveit_msgs_importable():
    """Verify that the ROS2 message types used by the translated code are available."""
    try:
        from moveit_msgs.msg import CollisionObject, Grasp, PlaceLocation
        from trajectory_msgs.msg import JointTrajectory
        from shape_msgs.msg import SolidPrimitive

        co = CollisionObject()
        assert hasattr(co, 'id')
        assert hasattr(co, 'header')
        assert hasattr(co, 'primitives')

        g = Grasp()
        assert hasattr(g, 'grasp_pose')
        assert hasattr(g, 'pre_grasp_approach')
        assert hasattr(g, 'post_grasp_retreat')

        pl = PlaceLocation()
        assert hasattr(pl, 'place_pose')
        assert hasattr(pl, 'pre_place_approach')

        jt = JointTrajectory()
        assert hasattr(jt, 'joint_names')

        sp = SolidPrimitive()
        assert hasattr(sp, 'type')
    except ImportError as e:
        pytest.skip(f"Required ROS2 message packages not installed: {e}")