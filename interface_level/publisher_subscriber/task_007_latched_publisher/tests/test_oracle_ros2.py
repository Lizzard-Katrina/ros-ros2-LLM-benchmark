import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pytest
import time


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.received_msgs = []

        # Subscriber
        self.subscription = self.create_subscription(
            String, 'latched_topic', self.callback, 10
        )

        # Publisher with transient_local (latched) QoS
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, 'latched_topic', qos)

    def callback(self, msg):
        self.received_msgs.append(msg.data)

    def publish(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)


@pytest.fixture(scope="module")
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_context):
    node = TestNode()
    yield node
    node.destroy_node()


def spin_until(condition_fn, node, timeout=3.0):
    start = time.time()
    while not condition_fn() and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)


# ----------------------------
# Test 1: Single message
# ----------------------------
def test_single_message_received(node):
    msg = "TEST_SINGLE"
    node.publish(msg)

    spin_until(lambda: msg in node.received_msgs, node)

    assert msg in node.received_msgs


# ----------------------------
# Test 2: Multiple messages
# ----------------------------
def test_multiple_messages_received(node):
    msgs = [f"MSG_{i}" for i in range(5)]
    for m in msgs:
        node.publish(m)

    spin_until(lambda: all(m in node.received_msgs for m in msgs), node)

    for m in msgs:
        assert m in node.received_msgs


# ----------------------------
# Test 3: Latched behavior
# ----------------------------
def test_latched_behavior_for_new_subscriber(node):
    msgs = [f"LATCH_{i}" for i in range(3)]
    for m in msgs:
        node.publish(m)

    spin_until(lambda: msgs[-1] in node.received_msgs, node)

    new_received = []

    def new_callback(msg):
        new_received.append(msg.data)

    qos = rclpy.qos.QoSProfile(depth=10)
    qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

    node.create_subscription(String, 'latched_topic', new_callback, qos)

    rclpy.spin_once(node, timeout_sec=0.5)

    assert msgs[-1] in new_received
