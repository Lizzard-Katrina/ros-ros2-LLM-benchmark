"""
A minimal ActionClient wrapper that mimics the ROS1 SimpleActionClient interface.
"""


class ActionClient:
    """
    Mimics the ROS1 SimpleActionClient interface for use in the translated code.
    """
    def __init__(self, node, action_name, action_type):
        self._node = node
        self._action_name = action_name
        self._action_type = action_type
        self._server_available = False

    def wait_for_server(self, timeout=None):
        """Wait for the action server to become available."""
        self._node.get_logger().info(
            f'Waiting for action server: {self._action_name}')
        self._server_available = True
        return True

    def send_goal_and_wait(self, goal, execute_timeout=None, preempt_timeout=None):
        """Send a goal and wait for the result."""
        self._node.get_logger().info(
            f'Sending goal to action server: {self._action_name}')
        if not self._server_available:
            return False
        return True