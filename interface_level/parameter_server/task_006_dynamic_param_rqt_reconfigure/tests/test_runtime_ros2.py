"""
Runtime test for the param_api module.

Launches a parameter server node, then uses ParamClient to interact with it,
verifying list_parameters, get_parameters, set_parameters, and describe_parameters.
"""
import os
import sys
import time
import signal
import subprocess
import threading
import pytest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# Ensure param_api.py (at package root) is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from param_api import ParamClient, AsyncServiceCallFailed, create_param_client


@pytest.fixture(scope='module')
def ros_setup():
    """Initialize rclpy once for the module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def param_server_process():
    """Launch the param server node as a subprocess."""
    env = os.environ.copy()
    proc = subprocess.Popen(
        [sys.executable, '-c', '''
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('test_param_server')
node.declare_parameter('test_param_str', 'hello')
node.declare_parameter('test_param_int', 42)
node.declare_parameter('test_param_float', 3.14)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
'''],
        env=env,
    )
    # Give the node time to start
    time.sleep(2.0)
    yield proc
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


@pytest.fixture(scope='module')
def client_node(ros_setup):
    """Create a client node for ParamClient to use."""
    node = Node('test_param_client_node')
    yield node
    node.destroy_node()


@pytest.fixture(scope='module')
def param_client(client_node, param_server_process):
    """Create a ParamClient targeting the test_param_server node."""
    # Spin the client node in a background thread so service calls can complete
    executor_thread = threading.Thread(
        target=rclpy.spin, args=(client_node,), daemon=True
    )
    executor_thread.start()

    pc = create_param_client(client_node, '/test_param_server')
    yield pc
    pc.close()


def test_list_parameters(param_client):
    """Test that list_parameters returns the declared parameter names."""
    names = param_client.list_parameters()
    assert 'test_param_str' in names
    assert 'test_param_int' in names
    assert 'test_param_float' in names


def test_get_parameters(param_client):
    """Test that get_parameters returns correct values."""
    params = param_client.get_parameters(['test_param_str', 'test_param_int', 'test_param_float'])
    param_dict = {p.name: p.value for p in params}
    assert param_dict['test_param_str'] == 'hello'
    assert param_dict['test_param_int'] == 42
    assert abs(param_dict['test_param_float'] - 3.14) < 0.001


def test_set_parameters(param_client):
    """Test that set_parameters can change a parameter value."""
    new_params = [Parameter('test_param_int', Parameter.Type.INTEGER, 100)]
    result = param_client.set_parameters(new_params)
    assert result is not None
    # Verify the change took effect
    params = param_client.get_parameters(['test_param_int'])
    assert params[0].value == 100


def test_describe_parameters(param_client):
    """Test that describe_parameters returns descriptors."""
    descriptors = param_client.describe_parameters(['test_param_str'])
    assert len(descriptors) == 1
    assert descriptors[0].name == 'test_param_str'


def test_create_param_client_function(client_node, param_server_process):
    """Test that create_param_client returns a ParamClient instance."""
    pc = create_param_client(client_node, '/test_param_server')
    assert isinstance(pc, ParamClient)
    pc.close()


def test_async_service_call_failed_exception():
    """Test the AsyncServiceCallFailed exception messages."""
    exc1 = AsyncServiceCallFailed(hint='timed out waiting for service')
    assert 'timed out waiting for service' in str(exc1)

    exc2 = AsyncServiceCallFailed(hint='the target node may not be spinning')
    assert 'the target node may not be spinning' in str(exc2)

    exc3 = AsyncServiceCallFailed()
    assert 'asynchronous service call failed' in str(exc3)


def test_service_timeout():
    """Test that calling a non-existent service raises AsyncServiceCallFailed with timeout hint."""
    rclpy2_context = rclpy.Context()
    rclpy2_context.init()
    try:
        node = Node('test_timeout_node', context=rclpy2_context)
        pc = ParamClient(node, '/nonexistent_node_xyz')
        with pytest.raises(AsyncServiceCallFailed, match='timed out waiting for service'):
            pc.list_parameters()
        pc.close()
        node.destroy_node()
    finally:
        rclpy2_context.shutdown()