"""
Runtime test for the migrated AMCL node's ROS 2 parameter system.
"""
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


@pytest.fixture(scope="module")
def amcl_process():
    """Launch the amcl_node as a subprocess."""
    proc = subprocess.Popen(
        ["ros2", "run", "task_007_navigation_stack_config", "amcl_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(2.0)  # Give the node time to start
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(rclpy_init):
    node = Node("test_amcl_params")
    yield node
    node.destroy_node()


def test_get_default_parameters(amcl_process, test_node):
    """Verify that declared parameters have the correct default values."""
    cli = test_node.create_client(GetParameters, "/amcl/get_parameters")
    assert cli.wait_for_service(timeout_sec=5.0), "get_parameters service not available"

    from rcl_interfaces.srv import GetParameters as GetParamsSrv
    req = GetParamsSrv.Request()
    req.names = ["min_particles", "max_particles", "odom_frame_id", "update_min_d"]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)
    assert future.done(), "Service call timed out"

    result = future.result()
    assert result is not None, "Service returned None"

    values = result.values
    assert len(values) == 4

    # min_particles default = 100
    assert values[0].type == ParameterType.PARAMETER_INTEGER
    assert values[0].integer_value == 100

    # max_particles default = 5000
    assert values[1].type == ParameterType.PARAMETER_INTEGER
    assert values[1].integer_value == 5000

    # odom_frame_id default = "odom"
    assert values[2].type == ParameterType.PARAMETER_STRING
    assert values[2].string_value == "odom"

    # update_min_d default = 0.2
    assert values[3].type == ParameterType.PARAMETER_DOUBLE
    assert abs(values[3].double_value - 0.2) < 1e-6


def test_set_valid_parameters(amcl_process, test_node):
    """Verify that valid parameter updates are accepted."""
    cli = test_node.create_client(SetParameters, "/amcl/set_parameters")
    assert cli.wait_for_service(timeout_sec=5.0), "set_parameters service not available"

    req = SetParameters.Request()
    param = Parameter()
    param.name = "min_particles"
    param.value = ParameterValue()
    param.value.type = ParameterType.PARAMETER_INTEGER
    param.value.integer_value = 200
    req.parameters = [param]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)
    assert future.done(), "Service call timed out"

    result = future.result()
    assert result is not None
    assert len(result.results) == 1
    assert result.results[0].successful is True


def test_reject_min_greater_than_max(amcl_process, test_node):
    """Verify that setting min_particles > max_particles is rejected."""
    cli = test_node.create_client(SetParameters, "/amcl/set_parameters")
    assert cli.wait_for_service(timeout_sec=5.0), "set_parameters service not available"

    req = SetParameters.Request()
    param = Parameter()
    param.name = "min_particles"
    param.value = ParameterValue()
    param.value.type = ParameterType.PARAMETER_INTEGER
    param.value.integer_value = 99999  # Way above max_particles (5000)
    req.parameters = [param]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)
    assert future.done(), "Service call timed out"

    result = future.result()
    assert result is not None
    assert len(result.results) == 1
    assert result.results[0].successful is False
    assert "min_particles" in result.results[0].reason.lower() or "max_particles" in result.results[0].reason.lower()


def test_set_string_parameter(amcl_process, test_node):
    """Verify that string parameters can be updated."""
    cli = test_node.create_client(SetParameters, "/amcl/set_parameters")
    assert cli.wait_for_service(timeout_sec=5.0), "set_parameters service not available"

    req = SetParameters.Request()
    param = Parameter()
    param.name = "odom_frame_id"
    param.value = ParameterValue()
    param.value.type = ParameterType.PARAMETER_STRING
    param.value.string_value = "odom_custom"
    req.parameters = [param]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)
    assert future.done(), "Service call timed out"

    result = future.result()
    assert result is not None
    assert len(result.results) == 1
    assert result.results[0].successful is True

    # Verify the value was actually set
    get_cli = test_node.create_client(GetParameters, "/amcl/get_parameters")
    assert get_cli.wait_for_service(timeout_sec=5.0)

    from rcl_interfaces.srv import GetParameters as GetParamsSrv
    get_req = GetParamsSrv.Request()
    get_req.names = ["odom_frame_id"]
    get_future = get_cli.call_async(get_req)
    rclpy.spin_until_future_complete(test_node, get_future, timeout_sec=5.0)
    assert get_future.done()
    get_result = get_future.result()
    assert get_result.values[0].string_value == "odom_custom"


def test_set_double_parameter(amcl_process, test_node):
    """Verify that double parameters can be updated."""
    cli = test_node.create_client(SetParameters, "/amcl/set_parameters")
    assert cli.wait_for_service(timeout_sec=5.0), "set_parameters service not available"

    req = SetParameters.Request()
    param = Parameter()
    param.name = "alpha1"
    param.value = ParameterValue()
    param.value.type = ParameterType.PARAMETER_DOUBLE
    param.value.double_value = 0.5
    req.parameters = [param]

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)
    assert future.done(), "Service call timed out"

    result = future.result()
    assert result is not None
    assert len(result.results) == 1
    assert result.results[0].successful is True