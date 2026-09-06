import subprocess
import time
import os
import signal
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


def test_babbler_publishes_hello_world():
    """Launch babbler node and verify it publishes 'hello world' messages on 'babble' topic."""
    rclpy.init()
    babbler_proc = None
    test_node = None
    try:
        # Launch babbler as subprocess
        babbler_proc = subprocess.Popen(
            ['ros2', 'run', 'task_004_talker_relay_listener', 'babbler'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        test_node = rclpy.create_node('test_babbler_listener')
        received_msgs = []

        def callback(msg):
            received_msgs.append(msg.data)

        sub = test_node.create_subscription(String, 'babble', callback, 10)

        # Spin for up to 5 seconds waiting for messages
        timeout = time.time() + 5.0
        while time.time() < timeout and len(received_msgs) < 3:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_msgs) >= 2, f"Expected at least 2 messages, got {len(received_msgs)}"
        # Check message format
        assert any("hello world" in m for m in received_msgs), \
            f"Expected 'hello world' in messages, got: {received_msgs}"

        # Verify incrementing count
        found_0 = any("hello world 0" in m for m in received_msgs)
        found_1 = any("hello world 1" in m for m in received_msgs)
        assert found_0 or found_1, f"Expected numbered messages, got: {received_msgs}"

    finally:
        if babbler_proc:
            babbler_proc.terminate()
            try:
                babbler_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                babbler_proc.kill()
                babbler_proc.wait(timeout=5)
        if test_node:
            test_node.destroy_node()
        rclpy.try_shutdown()


def test_add_two_ints_service():
    """Launch server, then use rclpy client to call the service and verify result."""
    rclpy.init()
    server_proc = None
    test_node = None
    try:
        # Launch server
        server_proc = subprocess.Popen(
            ['ros2', 'run', 'task_004_talker_relay_listener', 'add_two_ints_server'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give server time to start
        time.sleep(2.0)

        # Use our own rclpy client to call the service
        test_node = rclpy.create_node('test_service_client')
        client = test_node.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service
        assert client.wait_for_service(timeout_sec=5.0), "Service 'add_two_ints' not available"

        # Send request
        request = AddTwoInts.Request()
        request.a = 3
        request.b = 5
        future = client.call_async(request)

        # Spin until complete
        timeout = time.time() + 5.0
        while not future.done() and time.time() < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert future.done(), "Service call did not complete in time"
        result = future.result()
        assert result is not None, "Service call returned None"
        assert result.sum == 8, f"Expected sum=8, got sum={result.sum}"

    finally:
        if server_proc:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()
                server_proc.wait(timeout=5)
        if test_node:
            test_node.destroy_node()
        rclpy.try_shutdown()


def test_client_executable_runs():
    """Launch server and client together and verify client output contains the sum."""
    server_proc = None
    client_proc = None
    try:
        # Launch server
        server_proc = subprocess.Popen(
            ['ros2', 'run', 'task_004_talker_relay_listener', 'add_two_ints_server'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give server time to start
        time.sleep(2.0)

        # Launch client with arguments
        client_proc = subprocess.Popen(
            ['ros2', 'run', 'task_004_talker_relay_listener', 'add_two_ints_client', '10', '20'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for client to finish
        stdout, stderr = client_proc.communicate(timeout=10)
        combined = stdout.decode() + stderr.decode()

        # The client should print the sum (30)
        assert '30' in combined, f"Expected '30' in output, got: {combined}"

    finally:
        if client_proc and client_proc.poll() is None:
            client_proc.terminate()
            try:
                client_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                client_proc.kill()
                client_proc.wait(timeout=5)
        if server_proc:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()
                server_proc.wait(timeout=5)