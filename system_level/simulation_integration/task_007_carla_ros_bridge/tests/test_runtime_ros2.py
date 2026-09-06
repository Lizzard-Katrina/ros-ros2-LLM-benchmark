#!/usr/bin/env python3
"""
Runtime tests for the CARLA ROS Bridge migration task.

Tests both:
1. The transforms.py coordinate conversion logic (carla_velocity_to_ros_twist)
2. The bridge.py synchronous mode update logic
3. A live ROS2 node interaction test
"""

import math
import re
import subprocess
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock, patch
from threading import Event

import pytest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ============================================================
# Path setup for importing the source files
# ============================================================
PACKAGE_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PACKAGE_ROOT))

# ============================================================
# Test 1: Transforms - carla_velocity_to_ros_twist without rotation
# ============================================================
def test_twist_without_rotation():
    """Test carla_velocity_to_ros_twist without rotation (basic LH->RH)."""
    from transforms import carla_velocity_to_ros_twist

    # Mock carla.Vector3D
    linear_vel = MagicMock()
    linear_vel.x = 5.0
    linear_vel.y = 3.0
    linear_vel.z = 1.0

    angular_vel = MagicMock()
    angular_vel.x = 90.0   # degrees
    angular_vel.y = 45.0   # degrees
    angular_vel.z = 30.0   # degrees

    twist = carla_velocity_to_ros_twist(linear_vel, angular_vel)

    # Linear: x stays, y negated, z stays
    assert twist.linear.x == pytest.approx(5.0)
    assert twist.linear.y == pytest.approx(-3.0)
    assert twist.linear.z == pytest.approx(1.0)

    # Angular: x = radians(x), y = -radians(y), z = -radians(z)
    assert twist.angular.x == pytest.approx(math.radians(90.0))
    assert twist.angular.y == pytest.approx(-math.radians(45.0))
    assert twist.angular.z == pytest.approx(-math.radians(30.0))


# ============================================================
# Test 2: Transforms - carla_velocity_to_ros_twist with rotation
# ============================================================
def test_twist_with_rotation():
    """Test carla_velocity_to_ros_twist with rotation uses carla_vector_to_ros_vector_rotated."""
    from transforms import carla_velocity_to_ros_twist

    linear_vel = MagicMock()
    linear_vel.x = 10.0
    linear_vel.y = 0.0
    linear_vel.z = 0.0

    angular_vel = MagicMock()
    angular_vel.x = 0.0
    angular_vel.y = 0.0
    angular_vel.z = 0.0

    # Mock a rotation object with roll=0, pitch=0, yaw=0
    rotation = MagicMock()
    rotation.roll = 0.0
    rotation.pitch = 0.0
    rotation.yaw = 0.0

    twist = carla_velocity_to_ros_twist(linear_vel, angular_vel, carla_rotation=rotation)

    # With identity rotation, linear should be (10, 0, 0)
    assert twist.linear.x == pytest.approx(10.0, abs=1e-6)
    assert twist.linear.y == pytest.approx(0.0, abs=1e-6)
    assert twist.linear.z == pytest.approx(0.0, abs=1e-6)

    # Angular should all be 0
    assert twist.angular.x == pytest.approx(0.0)
    assert twist.angular.y == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.0)


# ============================================================
# Test 3: Transforms - angular handedness inversion
# ============================================================
def test_angular_handedness():
    """Verify angular Y and Z are negated for LH->RH conversion."""
    from transforms import carla_velocity_to_ros_twist

    linear_vel = MagicMock()
    linear_vel.x = 0.0
    linear_vel.y = 0.0
    linear_vel.z = 0.0

    angular_vel = MagicMock()
    angular_vel.x = 10.0
    angular_vel.y = 20.0
    angular_vel.z = 30.0

    twist = carla_velocity_to_ros_twist(linear_vel, angular_vel)

    # X is positive (no inversion)
    assert twist.angular.x > 0
    # Y and Z are negated
    assert twist.angular.y < 0
    assert twist.angular.z < 0


# ============================================================
# Test 4: Bridge - synchronous mode update sequence
# ============================================================
def test_synchronous_mode_update_sequence():
    """Test that _synchronous_mode_update calls in correct order."""
    sys.path.insert(0, str(PACKAGE_ROOT))

    # We need to mock the heavy dependencies
    call_order = []

    class MockTimestamp:
        elapsed_seconds = 1.0

    class MockSnapshot:
        timestamp = MockTimestamp()
        frame = 42

    class MockEgoVehicle:
        pass

    class MockActorFactory:
        actors = {}

        def update_available_objects(self):
            call_order.append('update_available_objects')

        def update_actor_states(self, frame_id, timestamp):
            call_order.append('update_actor_states')

    class MockWorldInfo:
        def update(self, frame_id, timestamp):
            call_order.append('world_info_update')

    class MockStatusPublisher:
        def set_synchronous_mode_running(self, val):
            pass

        def set_frame(self, frame):
            call_order.append('set_frame')

    class MockDebugHelper:
        pass

    class MockCarlaWorld:
        def tick(self):
            call_order.append('tick')
            return 42

        def get_snapshot(self):
            call_order.append('get_snapshot')
            return MockSnapshot()

    # Import bridge
    from bridge import CarlaRosBridge

    bridge = CarlaRosBridge.__new__(CarlaRosBridge)
    bridge.shutdown = Event()
    bridge.carla_world = MockCarlaWorld()
    bridge.actor_factory = MockActorFactory()
    bridge.world_info = MockWorldInfo()
    bridge.status_publisher = MockStatusPublisher()
    bridge.parameters = {
        'synchronous_mode_wait_for_vehicle_control_command': False
    }
    bridge.carla_run_state = 1  # PLAY
    bridge.carla_control_queue = __import__('queue').Queue()
    bridge._expected_ego_vehicle_control_command_ids = []
    bridge._expected_ego_vehicle_control_command_ids_lock = __import__('threading').Lock()
    bridge._all_vehicle_control_commands_received = Event()
    bridge.ros_timestamp = None

    # Make it run once then shutdown
    original_process = bridge.process_run_state

    call_count = [0]
    def mock_process():
        call_count[0] += 1
        if call_count[0] > 1:
            bridge.shutdown.set()

    bridge.process_run_state = mock_process

    # Run the synchronous mode update
    bridge._synchronous_mode_update()

    # Verify order
    assert 'update_available_objects' in call_order, "actor_factory.update_available_objects() must be called"
    assert 'tick' in call_order, "carla_world.tick() must be called"
    assert 'get_snapshot' in call_order, "carla_world.get_snapshot() must be called"

    tick_idx = call_order.index('tick')
    snapshot_idx = call_order.index('get_snapshot')
    factory_idx = call_order.index('update_available_objects')

    assert factory_idx < tick_idx, "actor_factory.update_available_objects() must be called before tick()"
    assert tick_idx < snapshot_idx, "tick() must be called before get_snapshot()"


# ============================================================
# Test 5: Static oracle tests - verify file content patterns
# ============================================================
def test_oracle_twist_linear_rotation_logic():
    """Verify linear velocity considers rotation coupling."""
    content = (PACKAGE_ROOT / "transforms.py").read_text()
    pattern = r"if\s+carla_rotation:.*carla_vector_to_ros_vector_rotated"
    assert re.search(pattern, content, re.DOTALL), \
        "Failed to find rotation-aware linear velocity transformation."


def test_oracle_twist_angular_unit_conversion():
    """Verify angular velocity unit conversion (deg to rad)."""
    content = (PACKAGE_ROOT / "transforms.py").read_text()
    pattern = r"ros_twist\.angular\.x\s*=\s*math\.radians\(carla_angular_velocity\.x\)"
    assert re.search(pattern, content), \
        "Angular velocity components must be converted using math.radians()."


def test_oracle_twist_angular_handedness_inversion():
    """Verify LH to RH coordinate inversion for angular Y/Z axes."""
    content = (PACKAGE_ROOT / "transforms.py").read_text()
    pattern_y = r"ros_twist\.angular\.y\s*=\s*-\s*math\.radians"
    pattern_z = r"ros_twist\.angular\.z\s*=\s*-\s*math\.radians"
    assert re.search(pattern_y, content) and re.search(pattern_z, content), \
        "Angular Y and Z axes must be inverted for system-level correctness."


def test_oracle_bridge_sync_tick_order():
    """Verify the sequence: tick() must happen before get_snapshot()."""
    content = (PACKAGE_ROOT / "bridge.py").read_text()
    tick_pos = content.find("self.carla_world.tick()")
    snapshot_pos = content.find("self.carla_world.get_snapshot()")
    assert tick_pos != -1 and snapshot_pos != -1, "Missing tick or snapshot logic in sync loop."
    assert tick_pos < snapshot_pos, \
        "Invalid sequence: tick() must precede get_snapshot() for frame consistency."


def test_oracle_bridge_clock_synchronization_call():
    """Verify clock is synchronized using the snapshot timestamp."""
    content = (PACKAGE_ROOT / "bridge.py").read_text()
    pattern = r"self\.update_clock\(\s*\w+_snapshot\.timestamp\s*\)"
    assert re.search(pattern, content), \
        "ROS system clock must be synchronized with world_snapshot timestamp."


def test_oracle_no_hardcoded_ego_id():
    """Ensure ego vehicle IDs are fetched dynamically from the factory."""
    content = (PACKAGE_ROOT / "bridge.py").read_text()
    forbidden_pattern = r"_expected_ego_vehicle_control_command_ids\s*=\s*\[\d+\]"
    assert not re.search(forbidden_pattern, content), \
        "Do not hardcode actor IDs; use actor_factory to identify EgoVehicles."


# ============================================================
# Test 6: Live ROS2 node interaction test
# ============================================================
def test_ros2_transforms_node_publishes_twist():
    """Launch the transforms_node and verify it publishes correct Twist messages."""
    rclpy.init()
    received_msgs = []
    node = None
    proc = None

    try:
        # Launch the node as a subprocess
        proc = subprocess.Popen(
            [sys.executable, '-m', 'task_007_carla_ros_bridge.transforms_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        time.sleep(1.0)

        # Create a test subscriber node
        node = rclpy.create_node('test_subscriber_node')
        sub = node.create_subscription(
            Twist,
            'carla_twist',
            lambda msg: received_msgs.append(msg),
            10
        )

        # Spin for up to 5 seconds waiting for messages
        timeout = time.time() + 5.0
        while time.time() < timeout and len(received_msgs) < 1:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert len(received_msgs) >= 1, "No Twist messages received from transforms_node"

        msg = received_msgs[0]
        # Verify angular values match the expected conversion pattern
        assert msg.angular.x == pytest.approx(math.radians(10.0), abs=1e-6)
        assert msg.angular.y == pytest.approx(-math.radians(20.0), abs=1e-6)
        assert msg.angular.z == pytest.approx(-math.radians(30.0), abs=1e-6)

    finally:
        if node:
            node.destroy_node()
        if proc:
            proc.terminate()
            proc.wait(timeout=5)
        rclpy.shutdown()


# ============================================================
# Test 7: Bridge update trigger with timestamp pattern
# ============================================================
def test_oracle_bridge_update_trigger_with_timestamp():
    """Verify global update is triggered with frame and elapsed time."""
    content = (PACKAGE_ROOT / "bridge.py").read_text()
    pattern = r"self\._update\(\s*frame\s*,\s*\w+\.timestamp\.elapsed_seconds\s*\)"
    assert re.search(pattern, content), \
        "Global _update() must be called with frame ID and elapsed simulation time."


def test_oracle_actor_factory_pre_tick_update():
    """Verify actor factory updates before the world ticks."""
    content = (PACKAGE_ROOT / "bridge.py").read_text()
    factory_str = "self.actor_factory.update_available_objects()"
    tick_pos = content.find("self.carla_world.tick()")
    factory_pos = content.find(factory_str)
    assert factory_pos != -1 and factory_pos < tick_pos, \
        "Actor factory must update available objects before ticking the world."