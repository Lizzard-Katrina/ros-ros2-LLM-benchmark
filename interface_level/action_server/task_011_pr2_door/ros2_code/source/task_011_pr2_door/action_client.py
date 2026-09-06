"""
Minimal ActionClient shim that mimics the ROS2 ActionClient interface
but works with our placeholder action types (no real action server needed
for the static oracle tests, but the API shape is correct).
"""
import rclpy
from rclpy.node import Node
import time


class ActionClient:
    """
    A simplified ActionClient compatible with the translated code.
    Signature: ActionClient(node, 'action_name', ActionType)
    """
    def __init__(self, node, action_name, action_type):
        self._node = node
        self._action_name = action_name
        self._action_type = action_type
        self._server_ready = False

    def wait_for_server(self, timeout_sec=5.0):
        """Wait for the action server (stub: just marks as ready)."""
        self._server_ready = True
        return True

    def send_goal_and_wait(self, goal, execute_timeout=None, preempt_timeout=None):
        """Send a goal and wait for result (stub: returns True)."""
        if not self._server_ready:
            return False
        return True

    def send_goal(self, goal):
        """Send a goal (stub)."""
        pass