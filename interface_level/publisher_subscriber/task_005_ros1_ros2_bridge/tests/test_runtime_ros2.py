#!/usr/bin/env python3
"""
Runtime test for task_005_ros1_ros2_bridge.
Launches the talker node via subprocess, then creates a test subscriber
to verify messages arrive on /chatter with expected content.
"""
import subprocess
import sys
import time
import threading

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestChatterSubscriber(Node):
    def __init__(self):
        super().__init__('test_chatter_subscriber')
        self.received_messages = []
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.callback,
            10)

    def callback(self, msg):
        self.received_messages.append(msg.data)


def test_talker_publishes_to_chatter():
    """Launch the talker node and verify it publishes on /chatter."""
    rclpy.init()
    talker_proc = None
    test_node = None
    try:
        # Launch the talker as a subprocess using ros2 run
        talker_proc = subprocess.Popen(
            [sys.executable, '-m', 'task_005_ros1_ros2_bridge.talker'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the talker a moment to start
        time.sleep(1.0)

        test_node = TestChatterSubscriber()

        # Spin for up to 5 seconds waiting for messages
        timeout = time.time() + 5.0
        while time.time() < timeout and len(test_node.received_messages) < 2:
            rclpy.spin_once(test_node, timeout_sec=0.5)

        # Assertions
        assert len(test_node.received_messages) >= 1, \
            f"Expected at least 1 message on /chatter, got {len(test_node.received_messages)}"
        assert test_node.received_messages[0] == 'hello from ros1', \
            f"Expected 'hello from ros1', got '{test_node.received_messages[0]}'"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        if talker_proc is not None:
            talker_proc.terminate()
            talker_proc.wait(timeout=5)
        rclpy.shutdown()


def test_talker_module_importable():
    """Verify the talker module can be imported and has main()."""
    from task_005_ros1_ros2_bridge import talker
    assert hasattr(talker, 'main'), "talker must define a main() function"
    assert callable(talker.main)


def test_listener_module_importable():
    """Verify the listener module can be imported and has main()."""
    from task_005_ros1_ros2_bridge import listener
    assert hasattr(listener, 'main'), "listener must define a main() function"
    assert callable(listener.main)


def test_no_rospy_in_talker():
    """Ensure no rospy references remain in the translated talker."""
    import inspect
    from task_005_ros1_ros2_bridge import talker
    source = inspect.getsource(talker)
    assert 'rospy' not in source, "Translated talker must not contain rospy references"


def test_string_message_construction():
    """Verify String message can be constructed and data assigned."""
    msg = String()
    msg.data = "test message"
    assert msg.data == "test message"


def test_publisher_on_chatter_topic():
    """Verify the TalkerNode creates a publisher on /chatter."""
    rclpy.init()
    try:
        from task_005_ros1_ros2_bridge.talker import TalkerNode
        node = TalkerNode()
        # Check that the node has a publisher
        publishers = node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace()
        )
        chatter_found = any(
            '/chatter' in name for name, types in publishers
        )
        assert chatter_found, \
            f"Expected publisher on /chatter, found: {publishers}"
        node.destroy_node()
    finally:
        rclpy.shutdown()