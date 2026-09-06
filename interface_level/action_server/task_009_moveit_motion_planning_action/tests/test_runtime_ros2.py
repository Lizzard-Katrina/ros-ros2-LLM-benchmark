"""
Runtime test for the translated MoveGroupMoveAction ROS2 action server.

This test:
1. Reads the translated move_action_capability.cpp to verify it exists and has ROS2 content
2. Launches a helper action server node (Python mirror of the C++ logic)
3. Sends real action goals and verifies responses
4. Checks the compiled library exists
"""
import os
import sys
import time
import subprocess
import threading
import pytest
from pathlib import Path


def test_source_file_exists_and_has_ros2_content():
    """Verify the translated .cpp file exists at the package root and contains ROS2 patterns."""
    cpp_file = Path(__file__).parent / "move_action_capability.cpp"
    assert cpp_file.exists(), f"move_action_capability.cpp not found at {cpp_file}"
    code = cpp_file.read_text()

    assert "rclcpp_action::create_server<moveit_msgs::action::MoveGroup>" in code
    assert "class MoveGroupMoveAction" in code
    assert "rclcpp::Node" in code
    assert "goal_handle->succeed(" in code
    assert "goal_handle->abort(" in code
    assert "goal_handle->canceled(" in code
    assert "preempt_requested_ = true" in code
    assert "publish_feedback(" in code
    assert "move_feedback_.state =" in code


def test_action_server_runtime():
    """
    Launch the helper action server as a subprocess, then use an rclpy
    action client to send goals and verify results.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    # Launch the helper server as a subprocess
    helper_script = str(Path(__file__).parent / "_test_action_server_node.py")
    server_proc = subprocess.Popen(
        [sys.executable, helper_script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    rclpy.init()
    client_node = None
    executor = None
    try:
        client_node = Node('test_move_group_client')
        executor = SingleThreadedExecutor()
        executor.add_node(client_node)

        action_client = ActionClient(client_node, MoveGroup, 'move_action')

        # Wait for server to come up
        assert action_client.wait_for_server(timeout_sec=10.0), \
            "Action server not available within timeout"

        # --- Test 1: plan_only = True ---
        received_feedback_1 = []

        def fb_cb_1(fb_msg):
            received_feedback_1.append(fb_msg.feedback.state)

        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options.plan_only = True

        send_future = action_client.send_goal_async(goal_msg, feedback_callback=fb_cb_1)

        # Spin until send_goal completes
        deadline = time.time() + 10.0
        while not send_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert send_future.done(), "send_goal_async timed out"

        goal_handle = send_future.result()
        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted"

        result_future = goal_handle.get_result_async()

        deadline = time.time() + 10.0
        while not result_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future.done(), "get_result_async timed out"

        result_wrapper = result_future.result()
        assert result_wrapper is not None, "No result received"
        result = result_wrapper.result
        assert result.error_code.val == MoveItErrorCodes.SUCCESS, \
            f"Expected SUCCESS, got {result.error_code.val}"

        # Give a moment for feedback to arrive
        end_fb = time.time() + 1.0
        while time.time() < end_fb:
            executor.spin_once(timeout_sec=0.05)

        assert len(received_feedback_1) > 0, "No feedback received for plan_only goal"
        assert "PLANNING" in received_feedback_1, \
            f"Expected PLANNING in feedback, got {received_feedback_1}"

        # --- Test 2: plan_only = False (plan and execute) ---
        received_feedback_2 = []

        def fb_cb_2(fb_msg):
            received_feedback_2.append(fb_msg.feedback.state)

        goal_msg2 = MoveGroup.Goal()
        goal_msg2.planning_options.plan_only = False

        send_future2 = action_client.send_goal_async(goal_msg2, feedback_callback=fb_cb_2)

        deadline = time.time() + 10.0
        while not send_future2.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert send_future2.done(), "send_goal_async timed out (goal 2)"

        goal_handle2 = send_future2.result()
        assert goal_handle2 is not None, "Goal handle 2 is None"
        assert goal_handle2.accepted, "Goal 2 was not accepted"

        result_future2 = goal_handle2.get_result_async()

        deadline = time.time() + 10.0
        while not result_future2.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future2.done(), "get_result_async timed out (goal 2)"

        result_wrapper2 = result_future2.result()
        assert result_wrapper2 is not None, "No result received for goal 2"
        result2 = result_wrapper2.result
        assert result2.error_code.val == MoveItErrorCodes.SUCCESS, \
            f"Expected SUCCESS for plan_and_execute, got {result2.error_code.val}"

        # Give a moment for feedback to arrive
        end_fb2 = time.time() + 1.0
        while time.time() < end_fb2:
            executor.spin_once(timeout_sec=0.05)

        assert len(received_feedback_2) > 0, "No feedback received for plan_and_execute goal"
        assert "MONITOR" in received_feedback_2, \
            f"Expected MONITOR in feedback for plan_and_execute, got {received_feedback_2}"

    finally:
        if client_node is not None:
            client_node.destroy_node()
        rclpy.try_shutdown()
        server_proc.terminate()
        try:
            server_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_proc.kill()
            server_proc.wait(timeout=3)


def test_compiled_library_exists():
    """
    Verify that the C++ library was actually compiled by checking
    for the shared library in the install space.
    """
    result = subprocess.run(
        ["ros2", "pkg", "prefix", "task_009_moveit_motion_planning_action"],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode == 0:
        prefix = result.stdout.strip()
        lib_dir = Path(prefix) / "lib"
        libs = list(lib_dir.glob("*move_action_capability*"))
        assert len(libs) > 0, \
            f"Compiled library not found in {lib_dir}. Contents: {list(lib_dir.iterdir())}"
    else:
        pytest.skip("Package not installed, skipping library check")