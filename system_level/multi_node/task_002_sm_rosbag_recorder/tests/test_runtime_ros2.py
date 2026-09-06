#!/usr/bin/env python3
"""
Runtime test for the ROS2 migrated talker node and recorder patterns.
Tests actual ROS2 communication by launching the talker and subscribing
to verify messages are received with expected content.
"""

import os
import re
import signal
import subprocess
import sys
import time
import threading

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestTalkerRuntime:
    """Test the talker node publishes correctly on the chatter topic."""

    def test_talker_publishes_hello_world(self):
        """Launch talker as subprocess and verify messages arrive on /chatter."""
        received_messages = []
        done_event = threading.Event()

        # Find the talker script
        pkg_root = Path(__file__).resolve().parent
        talker_script = pkg_root / 'talker.py'
        if not talker_script.exists():
            talker_script = pkg_root / 'scripts' / 'talker.py'

        assert talker_script.exists(), f"talker.py not found at {talker_script}"

        # Launch talker as subprocess
        proc = subprocess.Popen(
            [sys.executable, str(talker_script)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ, 'PYTHONUNBUFFERED': '1'}
        )

        test_node = None
        try:
            test_node = rclpy.create_node('test_talker_subscriber')

            def msg_callback(msg):
                received_messages.append(msg.data)
                if len(received_messages) >= 3:
                    done_event.set()

            sub = test_node.create_subscription(
                String, 'chatter', msg_callback, 10
            )

            # Spin in a thread to receive messages
            spin_thread = threading.Thread(
                target=lambda: _spin_until_event(test_node, done_event, timeout=8.0),
                daemon=True
            )
            spin_thread.start()
            spin_thread.join(timeout=10.0)

            # Verify we received messages
            assert len(received_messages) >= 1, \
                f"Expected at least 1 message on /chatter, got {len(received_messages)}"

            # Verify message content matches expected pattern: "hello world <timestamp>"
            for msg_data in received_messages:
                assert msg_data.startswith("hello world"), \
                    f"Message should start with 'hello world', got: {msg_data}"
                # Verify there's a numeric timestamp after "hello world "
                parts = msg_data.split("hello world ")
                assert len(parts) == 2, f"Unexpected message format: {msg_data}"
                timestamp_str = parts[1]
                try:
                    ts = float(timestamp_str)
                    assert ts > 0, f"Timestamp should be positive, got {ts}"
                except ValueError:
                    pytest.fail(f"Timestamp portion is not a valid float: {timestamp_str}")

        finally:
            if test_node is not None:
                test_node.destroy_node()
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3)


class TestRecorderStaticPatterns:
    """Verify the recorder.cpp contains correct ROS2 patterns."""

    def _get_recorder_path(self):
        pkg_root = Path(__file__).resolve().parent
        return pkg_root / 'recorder.cpp'

    def test_recorder_file_exists(self):
        assert self._get_recorder_path().exists(), "recorder.cpp not found"

    def test_generic_subscription_pattern(self):
        content = self._get_recorder_path().read_text()
        patterns = [r"generic_subscription", r"SerializedMessage"]
        assert any(re.search(p, content, re.IGNORECASE) for p in patterns), \
            "Recorder should use generic subscription or SerializedMessage"

    def test_qos_best_effort(self):
        content = self._get_recorder_path().read_text()
        qos_patterns = r"(SensorDataQoS|best_effort|BEST_EFFORT)"
        assert re.search(qos_patterns, content, re.IGNORECASE), \
            "Missing QoS best_effort/SensorDataQoS pattern"

    def test_node_clock_usage(self):
        content = self._get_recorder_path().read_text()
        assert re.search(r"this->now\(\)", content), \
            "Must use this->now() for ROS Domain Clock timestamps"
        assert "std::chrono::system_clock" not in content, \
            "Should not use system clock"

    def test_no_ros1_artifacts(self):
        content = self._get_recorder_path().read_text()
        assert not re.search(r"\bros::init\b", content), "Found ros::init"
        assert not re.search(r"\bros::NodeHandle\b", content), "Found ros::NodeHandle"


class TestTalkerStaticPatterns:
    """Verify the talker.py contains correct ROS2 patterns."""

    def _get_talker_path(self):
        pkg_root = Path(__file__).resolve().parent
        return pkg_root / 'talker.py'

    def test_talker_file_exists(self):
        assert self._get_talker_path().exists(), "talker.py not found"

    def test_declare_parameter(self):
        content = self._get_talker_path().read_text()
        assert "declare_parameter" in content

    def test_get_parameter(self):
        content = self._get_talker_path().read_text()
        assert "get_parameter" in content

    def test_create_timer(self):
        content = self._get_talker_path().read_text()
        assert "create_timer" in content
        assert re.search(r"0\.1", content), "Timer should use 0.1s period (10Hz)"

    def test_clock_usage(self):
        content = self._get_talker_path().read_text()
        assert re.search(r"get_clock\(.*?\)\.now\(.*?\)", content), \
            "Must use get_clock().now() for ROS clock"

    def test_no_rospy(self):
        content = self._get_talker_path().read_text()
        assert not re.search(r"\brospy\b", content), "Found legacy rospy reference"


def _spin_until_event(node, event, timeout=5.0):
    """Spin a node until an event is set or timeout expires."""
    end_time = time.time() + timeout
    while not event.is_set() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)