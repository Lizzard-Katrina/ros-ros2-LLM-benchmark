#!/usr/bin/env python3
"""
Runtime test for task_001_simple_pub_sub.
Exercises the actual translated talker and listener nodes.
"""
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@pytest.fixture(scope="module", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestTalkerNode:
    """Test that the talker node publishes on 'chatter'."""

    def test_talker_publishes(self):
        # Launch the talker as a subprocess using ros2 run
        talker_proc = subprocess.Popen(
            [sys.executable, "-c",
             "from task_001_simple_pub_sub.talker import main; main()"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        received_msgs = []

        class _TestSubscriber(Node):
            def __init__(self):
                super().__init__('test_talker_subscriber')
                self.sub = self.create_subscription(
                    String, 'chatter', self._cb, 10)

            def _cb(self, msg):
                received_msgs.append(msg.data)

        test_node = _TestSubscriber()
        try:
            timeout = time.time() + 10.0
            while time.time() < timeout and len(received_msgs) < 2:
                rclpy.spin_once(test_node, timeout_sec=0.1)

            assert len(received_msgs) >= 1, \
                f"Expected at least 1 message on 'chatter', got {len(received_msgs)}"
            # Check message content pattern
            assert any('Hello world' in m for m in received_msgs), \
                f"Expected 'Hello world' in messages, got {received_msgs}"
        finally:
            test_node.destroy_node()
            talker_proc.terminate()
            talker_proc.wait(timeout=5)


class TestListenerNode:
    """Test that the listener node subscribes to 'chatter'."""

    def test_listener_receives(self):
        # Launch the listener as a subprocess
        listener_proc = subprocess.Popen(
            [sys.executable, "-c",
             "from task_001_simple_pub_sub.listener import main; main()"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Create a publisher node to send a message
        pub_node = Node('test_listener_publisher')
        pub = pub_node.create_publisher(String, 'chatter', 10)

        try:
            # Give listener time to start and subscribe
            time.sleep(2.0)

            # Publish several messages
            for i in range(5):
                msg = String()
                msg.data = f'Test message {i}'
                pub.publish(msg)
                rclpy.spin_once(pub_node, timeout_sec=0.1)
                time.sleep(0.3)

            # Give listener time to process
            time.sleep(1.0)

            # Read stdout from listener to verify it received messages
            listener_proc.terminate()
            stdout, _ = listener_proc.communicate(timeout=5)
            output = stdout.decode('utf-8', errors='replace')

            assert 'I heard' in output, \
                f"Listener did not print expected output. Got: {output}"
        finally:
            pub_node.destroy_node()
            if listener_proc.poll() is None:
                listener_proc.terminate()
                listener_proc.wait(timeout=5)


class TestNodeStructure:
    """Test that the node classes are properly structured."""

    def test_talker_is_node_subclass(self):
        from task_001_simple_pub_sub.talker import TalkerNode
        assert issubclass(TalkerNode, Node)

    def test_talker_node_name(self):
        from task_001_simple_pub_sub.talker import TalkerNode
        node = TalkerNode()
        try:
            assert node.get_name() == 'talker'
            # Verify publisher exists by checking the node's publishers list
            # get_publisher_names_and_types_by_node requires the graph to be
            # populated; instead check the topic list directly
            topic_names = [t[0] for t in node.get_topic_names_and_types()]
            assert '/chatter' in topic_names, \
                f"Expected '/chatter' in {topic_names}"
        finally:
            node.destroy_node()

    def test_listener_is_node_subclass(self):
        from task_001_simple_pub_sub.listener import ListenerNode
        assert issubclass(ListenerNode, Node)

    def test_listener_node_name(self):
        from task_001_simple_pub_sub.listener import ListenerNode
        node = ListenerNode()
        try:
            assert node.get_name() == 'listener'
            # Verify subscription exists by checking the subscription attribute
            assert hasattr(node, 'subscription'), \
                "ListenerNode should have a 'subscription' attribute"
            assert node.subscription is not None, \
                "ListenerNode.subscription should not be None"
            # Check the subscription's topic name
            assert node.subscription.topic_name == '/chatter', \
                f"Expected subscription on '/chatter', got '{node.subscription.topic_name}'"
        finally:
            node.destroy_node()