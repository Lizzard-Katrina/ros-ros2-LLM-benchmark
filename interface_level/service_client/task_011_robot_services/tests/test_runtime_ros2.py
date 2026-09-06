#!/usr/bin/env python3
"""
Runtime test for task_011_robot_services: Camera Reconfigure Client Migration.

This test:
1. Starts a mock camera driver node (head_camera/driver) with auto_exposure
   and auto_white_balance parameters.
2. Imports and uses CameraReconfigure from the translated camera_reconfigure.py
   to call disable_auto() and enable_auto().
3. Verifies the parameters on the mock node actually changed.
"""
import subprocess
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def mock_driver_process():
    """Launch the mock camera driver node as a subprocess."""
    import os
    helper_path = os.path.join(os.path.dirname(__file__), '_test_helper_node.py')
    proc = subprocess.Popen(
        [sys.executable, helper_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    # Give it time to start
    time.sleep(2.0)
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def get_parameter_values(test_node, param_names, timeout=5.0):
    """Use GetParameters service to read current values from mock driver."""
    client = test_node.create_client(
        GetParameters,
        '/head_camera/driver/get_parameters'
    )
    assert client.wait_for_service(timeout_sec=timeout), \
        "GetParameters service not available"

    request = GetParameters.Request()
    request.names = param_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=timeout)
    assert future.done(), "GetParameters call did not complete"
    result = future.result()
    assert result is not None, "GetParameters returned None"
    return {name: val.bool_value for name, val in zip(param_names, result.values)}


def test_disable_auto(mock_driver_process):
    """Test that disable_auto sets both parameters to False."""
    # Import the actual translated module
    import importlib.util
    import os
    spec = importlib.util.spec_from_file_location(
        "camera_reconfigure",
        os.path.join(os.path.dirname(__file__), "camera_reconfigure.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # We need a separate rclpy context for the CameraReconfigure since it calls rclpy.create_node
    # But rclpy is already initialized. The module's class uses rclpy.create_node directly.
    reconfigure = mod.CameraReconfigure()
    try:
        reconfigure.disable_auto()
    finally:
        reconfigure.destroy()

    # Now verify the parameters on the mock driver
    test_node = rclpy.create_node('test_verifier_disable')
    try:
        values = get_parameter_values(test_node, ['auto_exposure', 'auto_white_balance'])
        assert values['auto_exposure'] is False, \
            f"Expected auto_exposure=False, got {values['auto_exposure']}"
        assert values['auto_white_balance'] is False, \
            f"Expected auto_white_balance=False, got {values['auto_white_balance']}"
    finally:
        test_node.destroy_node()


def test_enable_auto(mock_driver_process):
    """Test that enable_auto sets both parameters to True."""
    import importlib.util
    import os
    spec = importlib.util.spec_from_file_location(
        "camera_reconfigure",
        os.path.join(os.path.dirname(__file__), "camera_reconfigure.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    reconfigure = mod.CameraReconfigure()
    try:
        reconfigure.enable_auto()
    finally:
        reconfigure.destroy()

    # Verify
    test_node = rclpy.create_node('test_verifier_enable')
    try:
        values = get_parameter_values(test_node, ['auto_exposure', 'auto_white_balance'])
        assert values['auto_exposure'] is True, \
            f"Expected auto_exposure=True, got {values['auto_exposure']}"
        assert values['auto_white_balance'] is True, \
            f"Expected auto_white_balance=True, got {values['auto_white_balance']}"
    finally:
        test_node.destroy_node()