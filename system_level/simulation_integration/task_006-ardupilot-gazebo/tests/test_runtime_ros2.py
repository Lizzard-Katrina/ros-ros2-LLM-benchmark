"""
Runtime test for task_006-ardupilot-gazebo.

This test:
1. Verifies SocketUDP compiles and works by running the test_socket_udp executable.
2. Performs static analysis on the translated source files to verify the oracle checks.
3. Does NOT reimplement any logic - it runs the actual compiled executable and reads
   the actual source files produced by the translation.
"""
import subprocess
import os
import re
import time
import pytest
from pathlib import Path


def find_package_root():
    """Find the package root directory."""
    # Try common locations
    candidates = [
        Path(__file__).resolve().parent,
        Path.cwd(),
    ]
    for c in candidates:
        if (c / "ArduPilotPlugin.cc").exists():
            return c
        if (c / "install" / "task_006-ardupilot-gazebo").exists():
            return c
    return Path(__file__).resolve().parent


def find_executable():
    """Find the test_socket_udp executable in the install space."""
    root = Path(__file__).resolve().parent
    # Search common install paths
    candidates = [
        root / "install" / "task_006-ardupilot-gazebo" / "lib" / "task_006-ardupilot-gazebo" / "test_socket_udp",
        root / "build" / "task_006-ardupilot-gazebo" / "test_socket_udp",
    ]
    # Also search via find
    for base in [root / "install", root / "build", root]:
        for p in base.rglob("test_socket_udp"):
            if p.is_file() and os.access(str(p), os.X_OK):
                candidates.insert(0, p)

    for c in candidates:
        if c.exists() and os.access(str(c), os.X_OK):
            return c
    return None


def find_source_file(name):
    """Find a source file in the package."""
    root = Path(__file__).resolve().parent
    # Direct location
    if (root / name).exists():
        return root / name
    # In share
    share = root / "install" / "task_006-ardupilot-gazebo" / "share" / "task_006-ardupilot-gazebo"
    if (share / name).exists():
        return share / name
    # Search
    for p in root.rglob(name):
        if p.is_file() and "build" not in str(p):
            return p
    for p in root.rglob(name):
        if p.is_file():
            return p
    return None


class TestSocketUDPExecutable:
    """Test that the SocketUDP code compiles and runs correctly."""

    def test_socket_udp_runs(self):
        """Run the test_socket_udp executable and verify it passes."""
        exe = find_executable()
        if exe is None:
            pytest.skip("test_socket_udp executable not found - build may not have completed")

        proc = subprocess.run(
            [str(exe)],
            capture_output=True,
            text=True,
            timeout=10
        )

        assert proc.returncode == 0, \
            f"test_socket_udp failed with rc={proc.returncode}\nstdout: {proc.stdout}\nstderr: {proc.stderr}"
        assert "ALL_PASS" in proc.stdout, \
            f"test_socket_udp did not produce ALL_PASS\nstdout: {proc.stdout}"

    def test_socket_sendrecv(self):
        """Verify the send/receive output."""
        exe = find_executable()
        if exe is None:
            pytest.skip("test_socket_udp executable not found")

        proc = subprocess.run(
            [str(exe)],
            capture_output=True,
            text=True,
            timeout=10
        )

        assert "PASS: sent and received" in proc.stdout
        assert "HELLO_ARDUPILOT" in proc.stdout


class TestSocketUDPSource:
    """Verify SocketUDP.cc source has correct implementations."""

    def test_pollin_uses_select(self):
        """Verify pollin uses select() with FD_ZERO/FD_SET."""
        path = find_source_file("SocketUDP.cc")
        assert path is not None, "SocketUDP.cc not found"
        content = path.read_text()

        assert "FD_ZERO" in content, "Must use FD_ZERO"
        assert "FD_SET" in content, "Must use FD_SET"
        assert re.search(r"select\s*\(\s*fd\s*\+\s*1", content), \
            "Must use select(fd + 1, ...)"

    def test_set_blocking_uses_fcntl(self):
        """Verify set_blocking uses fcntl with O_NONBLOCK."""
        path = find_source_file("SocketUDP.cc")
        assert path is not None
        content = path.read_text()

        assert "O_NONBLOCK" in content, "Must use O_NONBLOCK"
        assert "fcntl" in content, "Must use fcntl"

    def test_recv_uses_msg_dontwait(self):
        """Verify recv uses MSG_DONTWAIT."""
        path = find_source_file("SocketUDP.cc")
        assert path is not None
        content = path.read_text()

        assert "MSG_DONTWAIT" in content
        assert "::recvfrom" in content

    def test_timeout_conversion(self):
        """Verify timeout ms to s/us conversion."""
        path = find_source_file("SocketUDP.cc")
        assert path is not None
        content = path.read_text()

        assert "timeout_ms / 1000" in content or "timeout_ms % 1000" in content


class TestArduPilotPluginSource:
    """Verify ArduPilotPlugin.cc source has correct logic."""

    def test_lockstep_logic(self):
        """Verify lockstep uses isLockStep and receivedFirstCmd."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None, "ArduPilotPlugin.cc not found"
        content = path.read_text()

        assert "isLockStep" in content, "Must reference isLockStep"
        assert "receivedFirstCmd" in content, "Must reference receivedFirstCmd"

    def test_pwm_normalization_logic(self):
        """Verify PWM normalization: (raw_cmd + offset) * multiplier."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        # The oracle expects: (raw_cmd + offset) * multiplier pattern
        # Check for the pattern in UpdateMotorCommands
        pattern = r"\(\s*\w+\s*\+\s*\w+\s*\)\s*\*\s*\w+"
        assert re.search(pattern, content), \
            "Must have (raw_cmd + offset) * multiplier pattern"

        # More specific check matching oracle
        pattern2 = r"\(\s*raw_cmd\s*\+\s*offset\s*\)\s*\*\s*multiplier"
        assert re.search(pattern2, content), \
            "Must have exact (raw_cmd + offset) * multiplier"

    def test_no_hardcoded_addresses(self):
        """Ensure no hardcoded 127.0.0.1 in the plugin source."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        assert '"127.0.0.1"' not in content, \
            "Must not hardcode 127.0.0.1 - use SDF fdm_addr instead"

    def test_pose_transform_consistency(self):
        """Verify wldAToBdyA transform composition."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        assert "wldAToBdyA" in content, "Must compute wldAToBdyA"
        assert "wldAToWldG" in content, "Must compute wldAToWldG"
        assert "wldGToBdyG" in content, "Must use wldGToBdyG"
        assert "bdyAToBdyG" in content, "Must use bdyAToBdyG"
        # Check the composition uses Inverse()
        assert "Inverse()" in content, "Must use Inverse() in transform chain"

    def test_servo_json_parsing(self):
        """Verify the plugin parses servos from JSON."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        assert re.search(r"sock\.recv", content) or re.search(r"sock\.", content), \
            "Must receive data from socket"
        assert 'FindMember("servos")' in content or 'HasMember("servos")' in content, \
            "Must parse servos JSON key"

    def test_joint_command_application(self):
        """Verify commands applied via SetComponentData."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        assert "SetComponentData" in content, "Must use SetComponentData"
        components = ["JointForceCmd", "JointVelocityCmd"]
        found = any(c in content for c in components)
        assert found, "Must reference joint command components"

    def test_failsafe_handling(self):
        """Verify PWM == 0 failsafe check."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        # Check for pwm > 0 or outputReady check
        assert re.search(r"if\s*\(\s*\w+\s*>\s*0\s*\)", content) or \
               "pwm > 0" in content or "outputReady" in content, \
            "Must check for failsafe (pwm > 0 or outputReady)"

    def test_no_ros1_remnants(self):
        """Ensure no ROS1 symbols."""
        path_plugin = find_source_file("ArduPilotPlugin.cc")
        path_socket = find_source_file("SocketUDP.cc")
        content = ""
        if path_plugin:
            content += path_plugin.read_text()
        if path_socket:
            content += path_socket.read_text()

        ros1_symbols = [
            r"ros/ros\.h",
            r"ros::NodeHandle",
            r"ros::Publisher",
            r"ros::Subscriber",
            r"ros::init",
            r"ros::Time"
        ]
        for pattern in ros1_symbols:
            assert not re.search(pattern, content), \
                f"ROS1 symbol detected: {pattern}"

    def test_fdm_address_not_hardcoded_in_private(self):
        """Verify fdm_address member is not initialized to 127.0.0.1."""
        path = find_source_file("ArduPilotPlugin.cc")
        assert path is not None
        content = path.read_text()

        # The fdm_address member should not be initialized to "127.0.0.1"
        # It should be empty or sourced from SDF
        lines = content.split('\n')
        for line in lines:
            if 'fdm_address' in line and '127.0.0.1' in line:
                # Check if it's in the member declaration (not in a comment)
                stripped = line.strip()
                if not stripped.startswith('//') and not stripped.startswith('*'):
                    assert False, \
                        f"fdm_address hardcoded to 127.0.0.1: {line}"