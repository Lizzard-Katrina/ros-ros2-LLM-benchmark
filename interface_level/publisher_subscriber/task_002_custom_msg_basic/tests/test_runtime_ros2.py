#!/usr/bin/env python3
"""
Runtime test for task_002_custom_msg_basic.
Launches the publisher node via ros2 run, then uses a test subscriber
to verify that Person messages arrive on /person_info with expected content.

Also verifies the subscriber node can be launched without error.
"""
import subprocess
import sys
import time
import os

import pytest


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


def _remove_local_shadow():
    """
    Ensure that a local task_002_custom_msg_basic/ directory doesn't shadow
    the installed (colcon) package that contains the generated .msg Python module.
    We do this by removing '.' and the package source dir from sys.path if present,
    and by clearing any cached partial import.
    """
    cwd = os.getcwd()
    # Remove cwd and '' from sys.path so local __init__.py doesn't shadow installed pkg
    for p in [cwd, '', '.']:
        while p in sys.path:
            sys.path.remove(p)
    # If already partially imported, remove it so re-import picks up the installed one
    for key in list(sys.modules.keys()):
        if key.startswith('task_002_custom_msg_basic'):
            del sys.modules[key]


def test_publisher_sends_person_messages():
    """Launch publisher_node via ros2 run and verify messages arrive."""
    _remove_local_shadow()

    import rclpy
    from rclpy.node import Node
    from task_002_custom_msg_basic.msg import Person

    pub_proc = subprocess.Popen(
        ["ros2", "run", "task_002_custom_msg_basic", "publisher_node.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    received_msgs = []

    class TestSubscriber(Node):
        def __init__(self):
            super().__init__("test_subscriber_node")
            self.sub = self.create_subscription(
                Person, "person_info", self.on_msg, 10
            )

        def on_msg(self, msg):
            received_msgs.append(msg)

    node = TestSubscriber()
    try:
        deadline = time.time() + 15.0
        while time.time() < deadline and len(received_msgs) == 0:
            rclpy.spin_once(node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No Person messages received on /person_info"
        msg = received_msgs[0]
        assert msg.name == "Alice"
        assert msg.age == 30
        assert abs(msg.height - 1.65) < 0.01
    finally:
        node.destroy_node()
        pub_proc.terminate()
        pub_proc.wait(timeout=5)


def test_subscriber_node_launches():
    """Launch subscriber_node via ros2 run and verify it starts without error."""
    sub_proc = subprocess.Popen(
        ["ros2", "run", "task_002_custom_msg_basic", "subscriber_node.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        # Give it a couple seconds to start
        time.sleep(3.0)
        # Check it's still running (hasn't crashed)
        ret = sub_proc.poll()
        assert ret is None, (
            f"subscriber_node exited prematurely with code {ret}. "
            f"stderr: {sub_proc.stderr.read().decode()}"
        )

        # Verify the node is visible in the ROS graph
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=5
        )
        assert "/person_subscriber" in result.stdout, (
            f"person_subscriber not found in node list: {result.stdout}"
        )
    finally:
        sub_proc.terminate()
        sub_proc.wait(timeout=5)


def test_person_message_fields():
    """Verify the Person message has the expected fields."""
    _remove_local_shadow()

    from task_002_custom_msg_basic.msg import Person

    msg = Person()
    msg.name = "Bob"
    msg.age = 25
    msg.height = 1.80

    assert msg.name == "Bob"
    assert msg.age == 25
    assert abs(msg.height - 1.80) < 0.01

    # Verify field names exist
    assert hasattr(msg, 'name')
    assert hasattr(msg, 'age')
    assert hasattr(msg, 'height')