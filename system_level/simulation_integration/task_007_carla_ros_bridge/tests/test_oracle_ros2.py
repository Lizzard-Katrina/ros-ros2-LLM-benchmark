import re
import pytest
from pathlib import Path

# Adjust paths to your project structure
TRANSFORMS_FILE = Path(__file__).resolve().parents[1] / "transforms.py"
BRIDGE_FILE = Path(__file__).resolve().parents[1] / "bridge.py"

def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# ==========================================
# Transforms.py Tests (Coordinate Coupling)
# ==========================================

def test_twist_linear_rotation_logic():
    """Verify linear velocity considers rotation coupling."""
    content = get_content(TRANSFORMS_FILE)
    # Match the branch where carla_rotation is used to rotate the vector
    pattern = r"if\s+carla_rotation:.*carla_vector_to_ros_vector_rotated"
    assert re.search(pattern, content, re.DOTALL), \
        "Failed to find rotation-aware linear velocity transformation."

def test_twist_angular_unit_conversion():
    """Verify angular velocity unit conversion (deg to rad)."""
    content = get_content(TRANSFORMS_FILE)
    # Match math.radians usage for angular components
    pattern = r"ros_twist\.angular\.x\s*=\s*math\.radians\(carla_angular_velocity\.x\)"
    assert re.search(pattern, content), \
        "Angular velocity components must be converted using math.radians()."

def test_twist_angular_handedness_inversion():
    """Verify LH to RH coordinate inversion for angular Y/Z axes."""
    content = get_content(TRANSFORMS_FILE)
    # Ensure negative signs are applied to Y and Z to match ROS convention
    pattern_y = r"ros_twist\.angular\.y\s*=\s*-\s*math\.radians"
    pattern_z = r"ros_twist\.angular\.z\s*=\s*-\s*math\.radians"
    assert re.search(pattern_y, content) and re.search(pattern_z, content), \
        "Angular Y and Z axes must be inverted for system-level correctness."

# ==========================================
# Bridge.py Tests (Synchronous Lockstep)
# ==========================================

def test_bridge_sync_tick_order():
    """Verify the sequence: tick() must happen before get_snapshot()."""
    content = get_content(BRIDGE_FILE)
    tick_pos = content.find("self.carla_world.tick()")
    snapshot_pos = content.find("self.carla_world.get_snapshot()")
    
    assert tick_pos != -1 and snapshot_pos != -1, "Missing tick or snapshot logic in sync loop."
    assert tick_pos < snapshot_pos, \
        "Invalid sequence: tick() must precede get_snapshot() for frame consistency."

def test_bridge_clock_synchronization_call():
    """Verify clock is synchronized using the snapshot timestamp."""
    content = get_content(BRIDGE_FILE)
    # Check for update_clock call with timestamp attribute
    pattern = r"self\.update_clock\(\s*\w+_snapshot\.timestamp\s*\)"
    assert re.search(pattern, content), \
        "ROS system clock must be synchronized with world_snapshot timestamp."

def test_bridge_update_trigger_with_timestamp():
    """Verify global update is triggered with frame and elapsed time."""
    content = get_content(BRIDGE_FILE)
    # Match self._update call with frame and elapsed_seconds
    pattern = r"self\._update\(\s*frame\s*,\s*\w+\.timestamp\.elapsed_seconds\s*\)"
    assert re.search(pattern, content), \
        "Global _update() must be called with frame ID and elapsed simulation time."

def test_actor_factory_pre_tick_update():
    """Verify actor factory updates before the world ticks."""
    content = get_content(BRIDGE_FILE)
    pattern = r"self\.actor_factory\.update_available_objects\(\)"
    tick_pos = content.find("self.carla_world.tick()")
    factory_pos = content.find(pattern)
    
    assert factory_pos != -1 and factory_pos < tick_pos, \
        "Actor factory must update available objects before ticking the world."

# ==========================================
# Negative Tests (System Constraints)
# ==========================================

def test_no_hardcoded_ego_id():
    """Ensure ego vehicle IDs are fetched dynamically from the factory."""
    content = get_content(BRIDGE_FILE)
    # Prevent hardcoding specific IDs like [1, 2, 3]
    forbidden_pattern = r"_expected_ego_vehicle_control_command_ids\s*=\s*\[\d+\]"
    assert not re.search(forbidden_pattern, content), \
        "Do not hardcode actor IDs; use actor_factory to identify EgoVehicles."
