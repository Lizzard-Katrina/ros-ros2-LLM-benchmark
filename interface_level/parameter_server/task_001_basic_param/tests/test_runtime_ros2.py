import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

from task_001_basic_param.srv import GetCachedParam


def _terminate_process(proc):
    if proc is None:
        return
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2.0)


def _spin_until_future(node, future, timeout_sec):
    deadline = time.time() + timeout_sec
    while rclpy.ok() and not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), "future did not complete before timeout"
    return future.result()


def _call_cache_service(node, client, key, use_cache=True, timeout_sec=3.0):
    request = GetCachedParam.Request()
    request.key = key
    request.use_cache = use_cache
    future = client.call_async(request)
    response = _spin_until_future(node, future, timeout_sec)
    assert response is not None
    return response


def _set_provider_double_parameter(node, client, name, value, timeout_sec=5.0):
    request = SetParameters.Request()
    parameter = ParameterMsg()
    parameter.name = name
    parameter.value.type = ParameterType.PARAMETER_DOUBLE
    parameter.value.double_value = value
    request.parameters.append(parameter)

    future = client.call_async(request)
    response = _spin_until_future(node, future, timeout_sec)
    assert response is not None
    assert response.results
    assert response.results[0].successful, response.results[0].reason
    return response


def test_cached_parameter_is_refreshed_by_ros2_parameter_event():
    package_root = Path(__file__).resolve().parent
    helper_path = package_root / "_test_helper_node.py"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    provider_proc = None
    cache_proc = None

    rclpy.init()
    test_node = Node("param_cache_runtime_test")

    try:
        provider_proc = subprocess.Popen(
            [sys.executable, str(helper_path)],
            cwd=str(package_root),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        cache_proc = subprocess.Popen(
            ["ros2", "run", "task_001_basic_param", "param_cache_node", "/param_provider"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        cache_client = test_node.create_client(GetCachedParam, "/get_cached_param")
        assert cache_client.wait_for_service(timeout_sec=8.0), "cache service did not become available"

        set_param_client = test_node.create_client(SetParameters, "/param_provider/set_parameters")
        assert set_param_client.wait_for_service(timeout_sec=8.0), "provider set_parameters service unavailable"

        initial = _call_cache_service(test_node, cache_client, "/robot/speed", use_cache=True)
        assert initial.success
        assert float(initial.value) == pytest.approx(1.5)

        _set_provider_double_parameter(test_node, set_param_client, "robot.speed", 2.5)

        deadline = time.time() + 6.0
        refreshed = None
        while time.time() < deadline:
            refreshed = _call_cache_service(test_node, cache_client, "/robot/speed", use_cache=True)
            if refreshed.success and abs(float(refreshed.value) - 2.5) < 1e-9:
                break
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert refreshed is not None
        assert refreshed.success
        assert float(refreshed.value) == pytest.approx(2.5)

    finally:
        test_node.destroy_node()
        rclpy.shutdown()
        _terminate_process(cache_proc)
        _terminate_process(provider_proc)