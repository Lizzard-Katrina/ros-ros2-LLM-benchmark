#!/usr/bin/env python3
"""
Runtime test for task_001_simple_service.
Launches the server node via subprocess, then uses the client class
in-process to call the service and verify the result.
"""

import subprocess
import sys
import time
import pytest

import rclpy
from rclpy.node import Node


def test_service_call_returns_correct_sum():
    """Start the server, call it with the client class, and verify the sum."""
    # Launch the server as a subprocess using ros2 run
    server_proc = subprocess.Popen(
        [sys.executable, '-c',
         'from task_001_simple_service.ros_server import main; main()'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    try:
        # Give the server a moment to start
        time.sleep(2.0)

        # Now use the actual client class from the translated module
        rclpy.init()
        try:
            from task_001_simple_service.ros_client import AddTwoIntsClient

            client_node = AddTwoIntsClient()
            try:
                # Test 2 + 3 = 5
                result = client_node.call_service(2, 3)
                assert result is not None, "Service call returned None"
                assert result.sum == 5, f"Expected 5, got {result.sum}"

                # Test 10 + 20 = 30
                result2 = client_node.call_service(10, 20)
                assert result2 is not None, "Service call returned None"
                assert result2.sum == 30, f"Expected 30, got {result2.sum}"

                # Test 0 + 0 = 0
                result3 = client_node.call_service(0, 0)
                assert result3 is not None, "Service call returned None"
                assert result3.sum == 0, f"Expected 0, got {result3.sum}"

                # Test negative numbers: -5 + 3 = -2
                result4 = client_node.call_service(-5, 3)
                assert result4 is not None, "Service call returned None"
                assert result4.sum == -2, f"Expected -2, got {result4.sum}"

            finally:
                client_node.destroy_node()
        finally:
            rclpy.shutdown()
    finally:
        server_proc.terminate()
        server_proc.wait(timeout=5)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])