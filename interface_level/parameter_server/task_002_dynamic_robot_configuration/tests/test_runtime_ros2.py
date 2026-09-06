"""
Runtime test for task_002_dynamic_robot_configuration.
Launches the turtlesim_node and verifies the background_color_rgb parameter
is correctly declared with the expected default values.
"""
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_parameter_declared_with_defaults():
    """Launch the node and verify background_color_rgb parameter has correct defaults."""
    # Launch the turtlesim_node as a subprocess
    proc = subprocess.Popen(
        ["ros2", "run", "task_002_dynamic_robot_configuration", "turtlesim_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    test_node = None
    try:
        # Give the node time to start up
        time.sleep(2.0)

        # Create a test node
        test_node = Node("test_param_checker")

        # Create a client for the GetParameters service on the turtlesim node
        client = test_node.create_client(
            GetParameters, "/turtlesim/get_parameters"
        )

        # Wait for the service to be available
        assert client.wait_for_service(timeout_sec=5.0), \
            "GetParameters service not available within timeout"

        # Build the request
        request = GetParameters.Request()
        request.names = ["background_color_rgb"]

        # Call the service
        future = client.call_async(request)

        # Spin until we get a response or timeout
        timeout = time.time() + 5.0
        while not future.done() and time.time() < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert future.done(), "Service call did not complete within timeout"

        result = future.result()
        assert result is not None, "Service call returned None"
        assert len(result.values) == 1, \
            f"Expected 1 parameter value, got {len(result.values)}"

        param_value = result.values[0]
        # Type 7 = PARAMETER_INTEGER_ARRAY in rcl_interfaces
        assert param_value.type == 7, \
            f"Expected integer array type (7), got type {param_value.type}"

        rgb = list(param_value.integer_array_value)
        assert rgb == [69, 86, 255], \
            f"Expected default RGB [69, 86, 255], got {rgb}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()