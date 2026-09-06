#!/usr/bin/env python3
"""
Runtime test for the CalibrationActionServer translated to ROS2.
Launches the action server node, sends a goal, and verifies feedback + result.
"""
import subprocess
import sys
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


def test_calibration_action_server():
    """Send a goal to the calibration action server and verify result and feedback."""
    # Start the action server as a subprocess
    server_proc = subprocess.Popen(
        [sys.executable, '-c',
         'import sys; sys.argv = ["calibration_action_server_stub"]; '
         'from calibration_action_server_stub import main; main()'],
        cwd=str(__import__('pathlib').Path(__file__).parent),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    rclpy.init()
    node = None
    try:
        # Give server time to start
        time.sleep(2.0)

        node = rclpy.create_node('test_calibration_client')

        from task_001_base_case.action import RobotCalibration
        action_client = ActionClient(node, RobotCalibration, 'robot_calibration')

        # Wait for action server
        assert action_client.wait_for_server(timeout_sec=10.0), \
            "Action server not available within timeout"

        # Send goal
        goal_msg = RobotCalibration.Goal()
        goal_msg.num_steps = 5
        goal_msg.calibration_type = 'full'

        feedbacks_received = []

        def feedback_callback(feedback_msg):
            feedbacks_received.append(feedback_msg.feedback)

        send_goal_future = action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )

        # Wait for goal acceptance
        deadline = time.time() + 10.0
        while not send_goal_future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert send_goal_future.done(), "Goal send did not complete in time"

        goal_handle = send_goal_future.result()
        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted"

        # Wait for result
        result_future = goal_handle.get_result_async()
        deadline = time.time() + 15.0
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert result_future.done(), "Result not received in time"

        result = result_future.result()
        assert result is not None, "Result is None"

        # Check result fields
        assert result.result.success is True, \
            f"Expected success=True, got {result.result.success}"
        assert 'completed' in result.result.message.lower() or \
               'success' in result.result.message.lower(), \
            f"Unexpected result message: {result.result.message}"

        # Check that we received some feedback
        assert len(feedbacks_received) > 0, "No feedback messages received"

        # Check last feedback is ~100%
        last_fb = feedbacks_received[-1]
        assert last_fb.percent_complete >= 99.0, \
            f"Expected ~100% completion, got {last_fb.percent_complete}"

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        server_proc.terminate()
        try:
            server_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_proc.kill()
            server_proc.wait(timeout=5)