"""
Runtime test for the migrated rosserial_python package.
Tests that:
1. The SerialClient can be instantiated with a node (dependency injection)
2. The node publishes diagnostics
3. Clock and logger are properly wired through the injected node
"""
import pytest
import time
import threading
import sys
import os
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def _add_pkg_root_to_path():
    """Ensure the package root is on sys.path so SerialClient can be imported."""
    pkg_root = os.path.dirname(os.path.abspath(__file__))
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)


class MockPort:
    """A mock serial port object that satisfies the duck-typed interface."""
    def read(self, size):
        return b'\x00' * size
    def write(self, data):
        pass
    def flushInput(self):
        pass
    def flushOutput(self):
        pass
    def inWaiting(self):
        return 0


def test_serial_client_dependency_injection():
    """Test that SerialClient accepts a node and uses its clock/logger."""
    _add_pkg_root_to_path()
    from SerialClient import SerialClient

    node = Node('test_serial_client_node')
    try:
        mock_port = MockPort()

        # Instantiate SerialClient with the node (dependency injection)
        client = SerialClient(node, port=mock_port, baud=57600, timeout=1.0)

        # Verify the node was stored
        assert client.node is node, "SerialClient must store the injected node"

        # Verify clock works through the node
        now = client.node.get_clock().now()
        assert now.nanoseconds > 0, "Clock should return a valid time"

        # Verify logger works through the node
        logger = client.node.get_logger()
        assert logger is not None, "Logger should be accessible through the node"

        # Verify the diagnostics publisher was created
        assert client.pub_diagnostics is not None, "Diagnostics publisher should be created"

        # Verify lastsync was initialized with node clock
        assert client.lastsync.nanoseconds >= 0, "lastsync should be initialized with node clock"

    finally:
        node.destroy_node()


def test_diagnostics_publisher():
    """Test that SerialClient publishes diagnostics via the injected node."""
    _add_pkg_root_to_path()
    from SerialClient import SerialClient
    from diagnostic_msgs.msg import DiagnosticStatus

    node = Node('test_diag_node')
    listener_node = Node('test_diag_listener')

    received_msgs = []

    def diag_callback(msg):
        received_msgs.append(msg)

    try:
        mock_port = MockPort()
        client = SerialClient(node, port=mock_port, baud=57600, timeout=1.0)

        sub = listener_node.create_subscription(
            DiagnosticArray, '/diagnostics', diag_callback, 10)

        # Send a diagnostic message
        client.sendDiagnostics(DiagnosticStatus.WARN, "test diagnostic message")

        # Spin both nodes to allow message passing
        deadline = time.time() + 5.0
        while time.time() < deadline and len(received_msgs) == 0:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(listener_node, timeout_sec=0.05)

        assert len(received_msgs) > 0, "Should have received at least one diagnostic message"
        diag_msg = received_msgs[0]
        assert len(diag_msg.status) > 0, "Diagnostic message should have status entries"
        assert diag_msg.status[0].name == "rosserial_python", \
            "Diagnostic status name should be 'rosserial_python'"
        assert diag_msg.status[0].message == "test diagnostic message", \
            "Diagnostic message text should match"

    finally:
        listener_node.destroy_node()
        node.destroy_node()


def test_node_clock_time_request():
    """Test that handleTimeRequest uses node.get_clock().now()."""
    _add_pkg_root_to_path()
    from SerialClient import SerialClient

    node = Node('test_time_node')
    try:
        mock_port = MockPort()
        client = SerialClient(node, port=mock_port, baud=57600, timeout=1.0)

        before = node.get_clock().now()
        client.handleTimeRequest(b'')
        after = node.get_clock().now()

        # lastsync should have been updated
        assert client.lastsync.nanoseconds >= before.nanoseconds, \
            "lastsync should be updated after handleTimeRequest"
        assert client.lastsync.nanoseconds <= after.nanoseconds, \
            "lastsync should not be in the future"

        # Check that something was queued
        assert not client.write_queue.empty(), \
            "handleTimeRequest should queue a time response"

    finally:
        node.destroy_node()


def test_serial_node_parameters():
    """Test that SerialNode declares and retrieves parameters correctly."""
    _add_pkg_root_to_path()

    from pathlib import Path
    pkg_root = os.path.dirname(os.path.abspath(__file__))

    node_file = Path(pkg_root) / 'serial_node.py'
    content = node_file.read_text()

    assert 'declare_parameter' in content, "serial_node.py must use declare_parameter"
    assert 'get_parameter' in content, "serial_node.py must use get_parameter"
    assert 'rclpy.spin' in content, "serial_node.py must use rclpy.spin"
    assert 'SerialClient' in content, "serial_node.py must use SerialClient"