import re
from pathlib import Path
import pytest

# Path resolution
ROOT_DIR = Path(__file__).resolve().parents[1]
PLUGIN_CPP = ROOT_DIR /"ArduPilotPlugin.cc"
SOCKET_CPP = ROOT_DIR /"SocketUDP.cc"

def get_content(path):
    return path.read_text(encoding='utf-8') if path.exists() else ""

# --- 1. SocketUDP Communication Semantics ---

def test_socket_pollin_logic():
    """Verify that SocketUDP implements non-blocking timeout check using select()."""
    content = get_content(SOCKET_CPP)
    # Check for select() setup: FD_ZERO, FD_SET, and the select call itself
    assert "FD_ZERO" in content and "FD_SET" in content, "SocketUDP must initialize file descriptor sets for select()."
    assert re.search(r"select\s*\(\s*fd\s*\+\s*1", content), "SocketUDP must use select() to monitor the file descriptor."
    # Check for correct timeout unit conversion (ms to s/us)
    assert "tv_sec = timeout_ms / 1000" in content or "timeout_ms % 1000" in content

def test_socket_recv_semantics():
    """Verify that recvfrom uses MSG_DONTWAIT to prevent simulation hangs."""
    content = get_content(SOCKET_CPP)
    # Ensure MSG_DONTWAIT is used to keep the Gazebo update loop fluid
    assert "MSG_DONTWAIT" in content, "SocketUDP::recv must use MSG_DONTWAIT for non-blocking operation."
    assert "::recvfrom" in content, "SocketUDP::recv must use the standard recvfrom system call."

# --- 2. ArduPilot Protocol & Lockstep (PreUpdate) ---

def test_preupdate_protocol_parsing():
    """Verify that PreUpdate correctly identifies and parses the 'servos' array from SITL JSON."""
    content = get_content(PLUGIN_CPP)
    # Look for the logic that receives data and looks for the servos key
    assert re.search(r"sock\.recv", content), "PreUpdate must attempt to receive data from the socket."
    assert "FindMember(\"servos\")" in content or "HasMember(\"servos\")" in content, \
        "The plugin must parse the 'servos' JSON key from ArduPilot SITL."

def test_lockstep_logic():
    """Verify that the plugin handles SITL lockstep synchronization."""
    content = get_content(PLUGIN_CPP)
    # Semantic check: if lockstep is on, simulation should wait for first command
    # Matches logic that checks isLockStep and receivedFirstCmd
    assert "isLockStep" in content and "receivedFirstCmd" in content, \
        "The plugin must implement lockstep logic to synchronize Gazebo with ArduPilot."

# --- 3. Control Mapping & Physical Execution (ApplyMotorForces) ---

def test_pwm_normalization_logic():
    """Verify that PWM values are mapped to physical commands using offset and multiplier."""
    content = get_content(PLUGIN_CPP)
    # Semantic check for: (pwm + offset) * multiplier or similar normalization
    # Allows for varying variable names but looks for the pattern of adding offset then multiplying
    pattern = r"\(\s*\w+\s*\+\s*\w+\.offset\s*\)\s*\*\s*\w+\.multiplier"
    assert re.search(pattern, content), "ApplyMotorForces must normalize PWM using 'offset' and 'multiplier' from SDF."

def test_joint_command_application():
    """Verify that commands are applied to Gazebo components (Force, Velocity, or Position)."""
    content = get_content(PLUGIN_CPP)
    # Check for the usage of Gazebo Sim ECM component setters
    # Must use JointForceCmd, JointVelocityCmd, or JointPositionCmd
    components = [
        "JointForceCmd",
        "JointVelocityCmd",
        "JointPositionCmd"
    ]
    found_component = any(comp in content for comp in components)
    assert found_component, "ApplyMotorForces must apply commands via Gazebo Sim Joint Command components."
    assert "SetComponentData" in content, "The plugin must use SetComponentData to apply control inputs to the ECM."

# --- 4. Safety and Robustness ---

def test_failsafe_handling():
    """Verify that the plugin handles zero-PWM (failsafe) conditions."""
    content = get_content(PLUGIN_CPP)
    # ArduPilot sends 0 PWM for disabled channels or failsafes
    assert re.search(r"if\s*\(\s*\w+\s*>\s*0\s*\)", content) or "pwm > 0" in content, \
        "The plugin should check if PWM > 0 before applying forces to avoid undefined behavior."

def test_no_hardcoded_addresses():
    """Ensure the plugin doesn't hardcode IP addresses, relying on SDF instead."""
    content = get_content(PLUGIN_CPP)
    # Search for common hardcoded IP strings
    bad_patterns = [r"\"127\.0\.0\.1\"", r"\"localhost\""]
    for p in bad_patterns:
        assert not re.search(p, content), f"Hardcoded IP detected: {p}. Use SDF <fdm_addr> instead."

def test_no_ros1_remnants():
    """Strictly ensure no legacy ROS 1 symbols or headers are present."""
    # This task is native Gazebo Sim / ArduPilot SITL, ROS 1 symbols indicate a dirty migration.
    content = get_content(PLUGIN_CPP) + get_content(SOCKET_CPP)
    ros1_symbols = [
        r"ros/ros\.h",
        r"ros::NodeHandle",
        r"ros::Publisher",
        r"ros::Subscriber",
        r"ros::init",
        r"ros::Time"
    ]
    for pattern in ros1_symbols:
        assert not re.search(pattern, content), f"Legacy ROS 1 symbol detected: {pattern}. This is a pure Gazebo Sim plugin."
