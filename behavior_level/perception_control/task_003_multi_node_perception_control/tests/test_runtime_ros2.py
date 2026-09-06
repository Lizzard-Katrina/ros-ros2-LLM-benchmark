"""
Runtime test for the translated kobuki_joystick ROS2 node.

We create a fake joystick device (a pipe-based file descriptor) and launch the
node with input_device pointing to it. Then we write joystick events into the
pipe and verify that the node publishes the expected cmd_vel and motor_power
messages.
"""

import os
import struct
import subprocess
import sys
import tempfile
import threading
import time

import pytest

# We need rclpy for the test subscriber
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


# js_event struct: __u32 time, __s16 value, __u8 type, __u8 number
# See <linux/joystick.h>
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# DS4 mappings from the code
DS4_L1 = 4  # button number
DS4_L3_Y = 1  # axis number
DS4_R3_X = 2  # axis number


def pack_js_event(time_ms, value, event_type, number):
    """Pack a js_event struct."""
    return struct.pack('<IhBB', time_ms, value, event_type, number)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class MessageCollector(Node):
    def __init__(self):
        super().__init__('test_collector')
        self.twist_msgs = []
        self.motor_msgs = []
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self._twist_cb, 10)
        self.motor_sub = self.create_subscription(
            UInt8, '/motor_power', self._motor_cb, 10)

    def _twist_cb(self, msg):
        self.twist_msgs.append(msg)

    def _motor_cb(self, msg):
        self.motor_msgs.append(msg)


def test_kobuki_joystick_publishes(ros_context):
    """
    Test that the kobuki_joystick node reads joystick events from a fake device
    and publishes cmd_vel and motor_power messages accordingly.
    """
    # Create a named pipe (FIFO) to act as a fake joystick device
    tmpdir = tempfile.mkdtemp()
    fifo_path = os.path.join(tmpdir, 'fake_js0')
    os.mkfifo(fifo_path)

    collector = MessageCollector()
    proc = None
    write_fd = None

    try:
        # Find the executable - it should be installed by colcon
        # Use ros2 run
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_003_multi_node_perception_control', 'kobuki_joystick',
             '--ros-args',
             '-p', f'input_device:={fifo_path}',
             '-p', 'scale_linear:=0.5',
             '-p', 'scale_angular:=1.5',
             '-r', 'cmd_vel:=/cmd_vel',
             '-r', 'motor_power:=/motor_power'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # The node will block on open() of the FIFO until we open the write end
        # Open write end (this unblocks the node's open())
        write_fd = os.open(fifo_path, os.O_WRONLY)

        # Give the node a moment to initialize
        time.sleep(1.0)

        # Spin collector to discover existing publishers
        for _ in range(20):
            rclpy.spin_once(collector, timeout_sec=0.05)

        # Step 1: Send L1 button press (enable)
        event_data = pack_js_event(100, 1, JS_EVENT_BUTTON, DS4_L1)
        os.write(write_fd, event_data)
        time.sleep(0.2)

        # Spin to collect motor_power ON message
        for _ in range(30):
            rclpy.spin_once(collector, timeout_sec=0.05)

        # Step 2: Send axis events (L3_Y and R3_X)
        # L3_Y value = -16384 => linear.x = -(-16384)/32767.0 * 0.5 = 0.25 (approx)
        event_data = pack_js_event(200, -16384, JS_EVENT_AXIS, DS4_L3_Y)
        os.write(write_fd, event_data)
        time.sleep(0.1)

        # R3_X value = 16384 => angular.z = -(16384)/32767.0 * 1.5 = -0.75 (approx)
        event_data = pack_js_event(300, 16384, JS_EVENT_AXIS, DS4_R3_X)
        os.write(write_fd, event_data)
        time.sleep(0.5)

        # Spin to collect cmd_vel messages
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if len(collector.twist_msgs) >= 2:
                break

        # Step 3: Send L1 button release (disable)
        event_data = pack_js_event(400, 0, JS_EVENT_BUTTON, DS4_L1)
        os.write(write_fd, event_data)
        time.sleep(0.5)

        # Spin to collect disable messages
        for _ in range(40):
            rclpy.spin_once(collector, timeout_sec=0.05)

        # --- Assertions ---

        # Check that we received motor_power messages
        # ON=1 should be first, OFF=0 should come after disable
        assert len(collector.motor_msgs) >= 1, \
            f"Expected at least 1 motor_power message, got {len(collector.motor_msgs)}"

        # First motor message should be ON (1)
        assert collector.motor_msgs[0].data == 1, \
            f"First motor_power should be ON (1), got {collector.motor_msgs[0].data}"

        # Check that we received cmd_vel messages while enabled
        assert len(collector.twist_msgs) >= 1, \
            f"Expected at least 1 cmd_vel message, got {len(collector.twist_msgs)}"

        # Check that at least one twist message has the expected approximate values
        found_expected_twist = False
        for msg in collector.twist_msgs:
            # linear.x should be approximately 0.25 (from -(-16384)/32767*0.5)
            # angular.z should be approximately -0.75 (from -(16384)/32767*1.5)
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                found_expected_twist = True
                # Check approximate values
                assert abs(msg.linear.x - 0.25) < 0.05, \
                    f"linear.x expected ~0.25, got {msg.linear.x}"
                assert abs(msg.angular.z - (-0.75)) < 0.05, \
                    f"angular.z expected ~-0.75, got {msg.angular.z}"
                break

        assert found_expected_twist, \
            "No cmd_vel message with expected non-zero values found"

        # If we got a second motor message, it should be OFF (0) from disable
        if len(collector.motor_msgs) >= 2:
            assert collector.motor_msgs[-1].data == 0, \
                f"Last motor_power should be OFF (0), got {collector.motor_msgs[-1].data}"

    finally:
        if write_fd is not None:
            try:
                os.close(write_fd)
            except OSError:
                pass
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3)
        collector.destroy_node()
        # Clean up FIFO
        try:
            os.unlink(fifo_path)
            os.rmdir(tmpdir)
        except OSError:
            pass


def test_source_file_exists_and_has_ros2_patterns():
    """
    Quick sanity check that the source file exists and contains key ROS2 patterns.
    This complements the static oracle tests.
    """
    from pathlib import Path

    # The source file should be at the package root
    pkg_root = Path(__file__).resolve().parent
    cpp_file = pkg_root / "kobuki_joystick.cpp"

    assert cpp_file.exists(), f"kobuki_joystick.cpp not found at {cpp_file}"

    code = cpp_file.read_text()
    assert 'rclcpp::init' in code, "Must contain rclcpp::init"
    assert 'rclcpp::shutdown' in code, "Must contain rclcpp::shutdown"
    assert 'create_publisher' in code, "Must contain create_publisher"
    assert 'geometry_msgs::msg::Twist' in code, "Must use geometry_msgs::msg::Twist"
    assert 'JS_EVENT_INIT' in code, "Must reference JS_EVENT_INIT"
    assert '32767' in code, "Must contain 32767 normalization constant"
    assert 'cmd_vel' in code, "Must reference cmd_vel topic"
    assert 'MotorPower::ON' in code, "Must reference MotorPower::ON"
    assert 'MotorPower::OFF' in code, "Must reference MotorPower::OFF"