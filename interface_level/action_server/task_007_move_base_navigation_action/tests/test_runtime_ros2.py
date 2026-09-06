"""
Runtime test for task_007_move_base_navigation_action.

Launches the stub move_base_node, subscribes to /move_base/current_goal,
and asserts that a PoseStamped message is received with expected values.
"""
import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_move_base_publishes_current_goal():
    """Verify the stub node publishes PoseStamped on move_base/current_goal."""
    proc = subprocess.Popen(
        ["ros2", "run", "task_007_move_base_navigation_action", "move_base_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    received_msgs = []

    class Listener(Node):
        def __init__(self):
            super().__init__("test_listener_007")
            self.sub = self.create_subscription(
                PoseStamped,
                "move_base/current_goal",
                self._cb,
                10,
            )

        def _cb(self, msg):
            received_msgs.append(msg)

    listener = None
    try:
        listener = Listener()
        # Spin for up to 5 seconds waiting for a message
        deadline = time.time() + 5.0
        while time.time() < deadline and len(received_msgs) == 0:
            rclpy.spin_once(listener, timeout_sec=0.1)

        assert len(received_msgs) > 0, "No PoseStamped received on move_base/current_goal"
        msg = received_msgs[0]
        assert msg.header.frame_id == "map"
        assert abs(msg.pose.position.x - 1.0) < 1e-3
        assert abs(msg.pose.position.y - 2.0) < 1e-3
        assert abs(msg.pose.orientation.w - 1.0) < 1e-3
    finally:
        if listener is not None:
            listener.destroy_node()
        proc.terminate()
        proc.wait(timeout=5)