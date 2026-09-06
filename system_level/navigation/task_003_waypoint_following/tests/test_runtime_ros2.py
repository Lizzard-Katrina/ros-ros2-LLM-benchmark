"""
Runtime test for the task_003_waypoint_following AddTwoInts action server.
Launches the compiled C++ action server, sends a goal via rclpy, and
asserts the result sum is correct.
"""
import subprocess
import time
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def test_add_two_ints_action_server():
    """Send a goal to the action server and verify the sum result."""
    server_proc = None
    try:
        # Launch the compiled action server executable
        server_proc = subprocess.Popen(
            ["ros2", "run", "task_003_waypoint_following", "add_two_ints_server"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the server time to start
        time.sleep(3)

        # Initialize rclpy for the test client
        rclpy.init()

        # We need to dynamically import the action type from the installed package
        from task_003_waypoint_following.action import TwoInts

        node = Node("test_action_client")
        action_client = ActionClient(node, TwoInts, "add_two_ints")

        # Wait for the action server to be available
        assert action_client.wait_for_server(timeout_sec=10.0), \
            "Action server did not become available within timeout"

        # Create goal
        goal_msg = TwoInts.Goal()
        goal_msg.a = 3
        goal_msg.b = 5

        # Send goal and wait for result
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()

        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted by the server"

        # Get the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=10.0)
        result = get_result_future.result()

        assert result is not None, "Result is None"
        assert result.result.sum == 8, f"Expected sum=8, got sum={result.result.sum}"

        # Test with different values
        goal_msg2 = TwoInts.Goal()
        goal_msg2.a = -10
        goal_msg2.b = 25

        send_goal_future2 = action_client.send_goal_async(goal_msg2)
        rclpy.spin_until_future_complete(node, send_goal_future2, timeout_sec=10.0)
        goal_handle2 = send_goal_future2.result()

        assert goal_handle2 is not None, "Second goal handle is None"
        assert goal_handle2.accepted, "Second goal was not accepted"

        get_result_future2 = goal_handle2.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future2, timeout_sec=10.0)
        result2 = get_result_future2.result()

        assert result2 is not None, "Second result is None"
        assert result2.result.sum == 15, f"Expected sum=15, got sum={result2.result.sum}"

        # Clean up rclpy resources
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()

    finally:
        if server_proc is not None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()
                server_proc.wait(timeout=5)
        # Ensure rclpy is shut down even on failure
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    pytest.main([__file__, "-v"])