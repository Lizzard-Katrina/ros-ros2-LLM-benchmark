#!/usr/bin/env python
"""
Runtime test for task_003_tutorial_baseline.
Exercises the SimMonitorState class from code.py by:
1. Starting a mock Gazebo set_model_state service.
2. Instantiating the real SimMonitorState and calling execute().
3. Asserting on the outcome and service interaction.
"""
import pytest
import threading
import time
import rclpy
from rclpy.node import Node


@pytest.fixture(scope="module", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def _get_srv_type():
    from task_003_tutorial_baseline.srv import SetModelState
    return SetModelState


class MockGazeboServiceNode(Node):
    def __init__(self):
        super().__init__('mock_gazebo_service_node')
        self.call_count = 0
        self.last_model_name = None
        SetModelState = _get_srv_type()
        self.srv = self.create_service(
            SetModelState,
            '/gazebo/set_model_state',
            self.handle_set_model_state
        )

    def handle_set_model_state(self, request, response):
        self.call_count += 1
        self.last_model_name = request.model_state.model_name
        response.success = True
        response.status_message = 'OK'
        return response


def test_sim_monitor_state_succeeds():
    """Test that SimMonitorState returns 'succeeded' after threshold and service call."""
    from task_003_tutorial_baseline_py.code import SimMonitorState

    service_node = MockGazeboServiceNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(0.5)

    state_node = None
    try:
        state_node = rclpy.create_node('test_sim_monitor_node')

        state = SimMonitorState(state_node, 'my_robot')

        class MockUD:
            time_threshold = 0.3
            target_color = None

        ud = MockUD()

        outcome = state.execute(ud)

        assert outcome == 'succeeded', f"Expected 'succeeded', got '{outcome}'"
        assert service_node.call_count == 1, \
            f"Expected service to be called once, got {service_node.call_count}"
        assert service_node.last_model_name == 'my_robot', \
            f"Expected model name 'my_robot', got '{service_node.last_model_name}'"

    finally:
        if state_node is not None:
            state_node.destroy_node()
        executor.shutdown()
        service_node.destroy_node()


def test_constructor_stores_node_and_model():
    """Test that the constructor properly stores node and model_name."""
    from task_003_tutorial_baseline_py.code import SimMonitorState

    service_node = MockGazeboServiceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)

    test_node = None
    try:
        test_node = rclpy.create_node('test_constructor_node')
        state = SimMonitorState(test_node, 'test_bot')

        assert state.node is test_node
        assert state._model_name == 'test_bot'
    finally:
        if test_node is not None:
            test_node.destroy_node()
        executor.shutdown()
        service_node.destroy_node()


def test_state_outcomes():
    """Verify the state has the expected outcomes."""
    from task_003_tutorial_baseline_py.code import SimMonitorState

    service_node = MockGazeboServiceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)

    test_node = None
    try:
        test_node = rclpy.create_node('test_outcomes_node')
        state = SimMonitorState(test_node, 'bot')

        assert 'succeeded' in state._outcomes
        assert 'preempted' in state._outcomes
        assert 'aborted' in state._outcomes
    finally:
        if test_node is not None:
            test_node.destroy_node()
        executor.shutdown()
        service_node.destroy_node()


def test_no_rospy_import():
    """Verify the translated code does not import rospy."""
    import pathlib
    pkg_code = pathlib.Path(__file__).resolve().parent / "task_003_tutorial_baseline_py" / "code.py"
    root_code = pathlib.Path(__file__).resolve().parent / "code.py"
    for p in [pkg_code, root_code]:
        if p.exists():
            content = p.read_text()
            assert "rospy" not in content, f"Legacy rospy detected in {p}"