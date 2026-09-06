#!/usr/bin/env python3
"""
Runtime test for the TurtleBot3 Patrol Action Server (ROS2).

This test:
1. Starts the patrol action server node in a subprocess.
2. Publishes fake odom so the server's turn() loop converges quickly.
3. Sends a square patrol goal via an action client.
4. Collects feedback messages and verifies the result string.
5. Cleans up properly.
"""

import subprocess
import sys
import time
import threading
import math
import os
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from task_002_turtlebot3_patrol_action.action import Patrol


def test_patrol_action_server():
    """Send a square patrol goal and verify result."""
    server_proc = None
    executor = None
    odom_node = None
    client_node = None
    spin_thread = None
    context = None

    try:
        # Start the server node as a subprocess
        server_proc = subprocess.Popen(
            ['ros2', 'run', 'task_002_turtlebot3_patrol_action', 'turtlebot3_patrol_server'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ},
        )
        time.sleep(3.0)  # Give server time to start

        # Use a dedicated context for the test
        context = rclpy.Context()
        context.init()

        odom_node = Node('test_odom_publisher', context=context)
        client_node = Node('test_patrol_client', context=context)

        qos = QoSProfile(depth=10)
        odom_pub = odom_node.create_publisher(Odometry, 'odom', qos)

        # Publish odom with rapidly changing yaw
        yaw_state = {'yaw': 0.0}

        def odom_timer_cb():
            msg = Odometry()
            msg.pose.pose.orientation.z = math.sin(yaw_state['yaw'] / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw_state['yaw'] / 2.0)
            odom_pub.publish(msg)
            yaw_state['yaw'] += 0.1

        odom_timer = odom_node.create_timer(0.02, odom_timer_cb)

        # Create executor and spin in background
        executor = MultiThreadedExecutor(context=context)
        executor.add_node(odom_node)
        executor.add_node(client_node)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        action_client = ActionClient(client_node, Patrol, 'turtlebot3')

        # Wait for action server
        assert action_client.wait_for_server(timeout_sec=15.0), \
            "Action server not available within timeout"

        # Build goal: mode=1 (square), distance=0.2, count=1
        goal_msg = Patrol.Goal()
        goal_msg.goal.x = 1.0   # square
        goal_msg.goal.y = 0.2   # travel distance
        goal_msg.goal.z = 1.0   # patrol count

        feedbacks = []

        def feedback_cb(feedback_msg):
            feedbacks.append(feedback_msg.feedback.state)

        # Send goal
        future = action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)

        # Wait for goal acceptance
        deadline = time.time() + 15.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.1)
        assert future.done(), "Goal send did not complete in time"

        goal_handle = future.result()
        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted"

        # Get result
        result_future = goal_handle.get_result_async()
        deadline = time.time() + 120.0  # generous timeout for patrol execution
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.2)

        assert result_future.done(), "Result not received within timeout"

        result = result_future.result()
        assert result is not None, "Result is None"
        result_str = result.result.result
        assert 'square patrol complete' in result_str.lower() or 'square' in result_str.lower(), \
            f"Unexpected result: {result_str}"

        # Verify we got some feedback
        assert len(feedbacks) > 0, "No feedback messages received"
        # Feedback should contain 'line' references
        assert any('line' in f for f in feedbacks), \
            f"Feedback did not contain expected 'line' messages: {feedbacks}"

    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if client_node is not None:
            try:
                client_node.destroy_node()
            except Exception:
                pass
        if odom_node is not None:
            try:
                odom_node.destroy_node()
            except Exception:
                pass
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                pass
        if server_proc is not None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-x'])