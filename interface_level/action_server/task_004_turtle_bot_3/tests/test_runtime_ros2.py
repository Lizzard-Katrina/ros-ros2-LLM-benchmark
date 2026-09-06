#!/usr/bin/env python3
"""
Runtime test for the head_action_server ROS2 node.
Launches the action server via subprocess, then sends a goal using an action client
and verifies feedback and result.
"""
import os
import sys
import time
import subprocess
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


def test_head_action_server_goal():
    """Send a goal to the head_action server and verify feedback + result."""

    # Start the action server as a subprocess using ros2 run
    env = os.environ.copy()
    server_proc = subprocess.Popen(
        ['ros2', 'run', 'task_004_turtle_bot_3', 'head_action_server.py'],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    try:
        rclpy.init()

        # Import the action type from the built package
        from task_004_turtle_bot_3.action import HeadPointing

        class TestActionClient(Node):
            def __init__(self):
                super().__init__('test_action_client')
                self._action_client = ActionClient(
                    self, HeadPointing, 'head_action'
                )
                self.feedbacks = []
                self.result_response = None
                self.goal_accepted = False

            def send_goal(self, target_frame):
                goal_msg = HeadPointing.Goal()
                goal_msg.target_frame = target_frame

                # Wait for server
                server_ready = self._action_client.wait_for_server(timeout_sec=15.0)
                if not server_ready:
                    raise RuntimeError('Action server not available')

                send_goal_future = self._action_client.send_goal_async(
                    goal_msg, feedback_callback=self.feedback_callback
                )
                send_goal_future.add_done_callback(self.goal_response_callback)

            def goal_response_callback(self, future):
                goal_handle = future.result()
                self.goal_accepted = goal_handle.accepted
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(self.result_callback)

            def result_callback(self, future):
                self.result_response = future.result()

            def feedback_callback(self, feedback_msg):
                self.feedbacks.append(feedback_msg.feedback)

        test_node = TestActionClient()

        try:
            # Give the server a moment to start
            time.sleep(3.0)

            test_node.send_goal('base_link')

            # Spin until we get a result or timeout
            timeout = time.time() + 25.0
            while time.time() < timeout:
                rclpy.spin_once(test_node, timeout_sec=0.1)
                if test_node.result_response is not None:
                    break

            # Assertions
            assert test_node.goal_accepted, "Goal was not accepted by the server"
            assert test_node.result_response is not None, "No result received (timeout)"

            result = test_node.result_response.result
            assert result.success is True, f"Expected success=True, got {result.success}"
            assert 'base_link' in result.message, \
                f"Expected 'base_link' in result message, got: {result.message}"

            # Check that we received feedback
            assert len(test_node.feedbacks) > 0, "No feedback messages received"

            # Check feedback progress values
            last_feedback = test_node.feedbacks[-1]
            assert last_feedback.progress > 0.0, \
                f"Expected progress > 0, got {last_feedback.progress}"

        finally:
            test_node.destroy_node()
            rclpy.shutdown()

    finally:
        server_proc.terminate()
        try:
            server_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_proc.kill()
            server_proc.wait(timeout=5)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])