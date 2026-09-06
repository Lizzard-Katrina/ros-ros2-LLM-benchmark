#!/usr/bin/env python3
"""
Runtime test for task_007_latched_publisher.
Exercises the actual translated node by importing its class and verifying
latched (transient local) publish/subscribe behavior.
"""
import pytest
import time
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


@pytest.fixture(scope="module")
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(condition_fn, node, timeout=5.0):
    start = time.time()
    while not condition_fn() and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)


class TestLatchedPublisher:
    """Test suite that imports and exercises the real LatchedPubSubNode."""

    def test_node_imports_and_creates(self, rclpy_context):
        """Test that the node class can be imported and instantiated."""
        from task_007_latched_publisher.latched_publisher import LatchedPubSubNode
        node = LatchedPubSubNode()
        try:
            assert node is not None
            assert node.get_name() == 'latched_pub_sub_node'
        finally:
            node.destroy_node()

    def test_single_message_received(self, rclpy_context):
        """Publish a single message via the node's publisher and verify receipt."""
        from task_007_latched_publisher.latched_publisher import LatchedPubSubNode
        node = LatchedPubSubNode()
        received = []

        try:
            qos = QoSProfile(depth=10)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            node.create_subscription(
                String, 'latched_topic', lambda msg: received.append(msg.data), qos
            )

            msg = String()
            msg.data = 'RUNTIME_TEST_SINGLE'
            node.publisher.publish(msg)

            spin_until(lambda: 'RUNTIME_TEST_SINGLE' in received, node, timeout=5.0)
            assert 'RUNTIME_TEST_SINGLE' in received
        finally:
            node.destroy_node()

    def test_multiple_messages(self, rclpy_context):
        """Publish multiple messages and verify all are received."""
        from task_007_latched_publisher.latched_publisher import LatchedPubSubNode
        node = LatchedPubSubNode()
        received = []

        try:
            qos = QoSProfile(depth=10)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            node.create_subscription(
                String, 'latched_topic', lambda msg: received.append(msg.data), qos
            )

            expected = [f'MULTI_{i}' for i in range(5)]
            for text in expected:
                msg = String()
                msg.data = text
                node.publisher.publish(msg)

            spin_until(lambda: all(m in received for m in expected), node, timeout=5.0)
            for m in expected:
                assert m in received
        finally:
            node.destroy_node()

    def test_latched_behavior(self, rclpy_context):
        """Verify transient local (latched) behavior: late subscriber gets the last message."""
        from task_007_latched_publisher.latched_publisher import LatchedPubSubNode
        node = LatchedPubSubNode()
        initial_received = []

        try:
            qos = QoSProfile(depth=10)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            # Initial subscriber
            node.create_subscription(
                String, 'latched_topic',
                lambda msg: initial_received.append(msg.data), qos
            )

            # Publish messages
            messages = [f'LATCH_RT_{i}' for i in range(3)]
            for text in messages:
                msg = String()
                msg.data = text
                node.publisher.publish(msg)

            # Wait for initial subscriber to get the last message
            spin_until(lambda: messages[-1] in initial_received, node, timeout=5.0)
            assert messages[-1] in initial_received

            # Now create a LATE subscriber — it should receive cached messages
            late_received = []
            node.create_subscription(
                String, 'latched_topic',
                lambda msg: late_received.append(msg.data), qos
            )

            # Wait until the late subscriber has received the last published message
            spin_until(lambda: messages[-1] in late_received, node, timeout=5.0)
            # The late subscriber should have received at least the last published message
            assert messages[-1] in late_received

        finally:
            node.destroy_node()

    def test_publisher_uses_transient_local(self, rclpy_context):
        """Verify the publisher QoS durability is TRANSIENT_LOCAL."""
        from task_007_latched_publisher.latched_publisher import LatchedPubSubNode
        node = LatchedPubSubNode()
        try:
            qos = node.publisher.qos_profile
            assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
        finally:
            node.destroy_node()