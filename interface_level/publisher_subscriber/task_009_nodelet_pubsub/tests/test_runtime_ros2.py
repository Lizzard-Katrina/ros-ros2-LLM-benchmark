"""
Runtime test for task_009_nodelet_pubsub.
Exercises the Plus node by publishing on 'in' and subscribing to 'out',
verifying the add-value logic.
"""
import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_plus_node_adds_value():
    """
    Launch the plus_node with parameter value:=10.0,
    publish 5.0 on 'in', expect 15.0 on 'out'.
    """
    proc = None
    test_node = None
    try:
        # Launch the plus_node executable with a parameter
        proc = subprocess.Popen(
            [
                "ros2", "run", "task_009_nodelet_pubsub", "plus_node",
                "--ros-args",
                "-p", "value:=10.0",
                "-r", "in:=test_in",
                "-r", "out:=test_out",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the node time to start
        time.sleep(2.0)

        # Create a test node
        test_node = Node("test_plus_client")

        publisher = test_node.create_publisher(Float64, "test_in", 10)

        received_msgs = []

        def on_msg(msg):
            received_msgs.append(msg.data)

        subscription = test_node.create_subscription(
            Float64, "test_out", on_msg, 10
        )

        # Publish messages and spin
        msg = Float64()
        msg.data = 5.0

        timeout = time.time() + 8.0
        while time.time() < timeout and len(received_msgs) == 0:
            publisher.publish(msg)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No messages received on 'test_out'"
        assert abs(received_msgs[0] - 15.0) < 1e-6, \
            f"Expected 15.0 but got {received_msgs[0]}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=5)


def test_plus_node_default_value():
    """
    Launch the plus_node with default value (0.0),
    publish 7.0 on 'in', expect 7.0 on 'out'.
    """
    proc = None
    test_node = None
    try:
        proc = subprocess.Popen(
            [
                "ros2", "run", "task_009_nodelet_pubsub", "plus_node",
                "--ros-args",
                "-r", "in:=test_in2",
                "-r", "out:=test_out2",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        time.sleep(2.0)

        test_node = Node("test_plus_default")

        publisher = test_node.create_publisher(Float64, "test_in2", 10)

        received_msgs = []

        def on_msg(msg):
            received_msgs.append(msg.data)

        subscription = test_node.create_subscription(
            Float64, "test_out2", on_msg, 10
        )

        msg = Float64()
        msg.data = 7.0

        timeout = time.time() + 8.0
        while time.time() < timeout and len(received_msgs) == 0:
            publisher.publish(msg)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No messages received on 'test_out2'"
        assert abs(received_msgs[0] - 7.0) < 1e-6, \
            f"Expected 7.0 but got {received_msgs[0]}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=5)