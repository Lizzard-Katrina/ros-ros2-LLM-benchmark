"""
Runtime test for task_013_slam_mapping_params.
Launches the slam_gmapping_node and verifies parameters via ros2 param get.
Each test uses a unique ROS_DOMAIN_ID to avoid cross-talk.
"""
import subprocess
import time
import os
import random
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters


def _kill_proc(proc):
    """Terminate and wait for a subprocess."""
    if proc is None:
        return
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def _call_get_parameters(executor, test_node, client, param_names, timeout=10.0):
    """Helper to call GetParameters service and return result."""
    assert client.wait_for_service(timeout_sec=timeout), \
        "Parameter service not available"

    request = GetParameters.Request()
    request.names = param_names

    future = client.call_async(request)
    start = time.time()
    while not future.done() and (time.time() - start) < timeout:
        executor.spin_once(timeout_sec=0.1)

    assert future.done(), "Service call timed out"
    result = future.result()
    assert result is not None, "Service returned None"
    return result


def test_slam_gmapping_parameters():
    """Launch the node, then query its parameters to verify defaults."""
    domain_id = random.randint(50, 200)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    proc = None
    context = rclpy.Context()
    executor = None
    test_node = None
    try:
        context.init(args=[], domain_id=domain_id)

        proc = subprocess.Popen(
            ["ros2", "run", "task_013_slam_mapping_params", "slam_gmapping_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        # Give the node time to start
        time.sleep(3.0)

        test_node = Node("test_param_client_defaults", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(test_node)

        client = test_node.create_client(
            GetParameters, "/slam_gmapping/get_parameters"
        )

        param_names = [
            "base_frame",
            "map_frame",
            "odom_frame",
            "delta",
            "particles",
            "temporalUpdate",
            "maxRange",
            "maxUrange",
            "srr",
            "srt",
            "str",
            "stt",
            "linearUpdate",
            "angularUpdate",
            "xmin",
            "ymin",
            "xmax",
            "ymax",
            "sigma",
            "kernelSize",
            "lstep",
            "astep",
            "iterations",
            "lsigma",
            "ogain",
            "lskip",
            "minimumScore",
            "resampleThreshold",
            "occ_thresh",
            "llsamplerange",
            "llsamplestep",
            "lasamplerange",
            "lasamplestep",
            "throttle_scans",
            "tf_delay",
        ]

        result = _call_get_parameters(executor, test_node, client, param_names)

        # Build a dict of name -> ParameterValue
        values = {}
        for i, name in enumerate(param_names):
            values[name] = result.values[i]

        # Check string params (type 4)
        assert values["base_frame"].type == 4
        assert values["base_frame"].string_value == "base_link"
        assert values["map_frame"].string_value == "map"
        assert values["odom_frame"].string_value == "odom"

        # Check double params (type 3)
        assert values["delta"].type == 3
        assert abs(values["delta"].double_value - 0.05) < 1e-6

        assert values["temporalUpdate"].type == 3
        assert abs(values["temporalUpdate"].double_value - (-1.0)) < 1e-6

        assert values["maxRange"].type == 3
        assert abs(values["maxRange"].double_value - 80.0) < 1e-6

        assert values["maxUrange"].type == 3
        assert abs(values["maxUrange"].double_value - 79.99) < 1e-6

        assert values["srr"].type == 3
        assert abs(values["srr"].double_value - 0.1) < 1e-6

        assert values["srt"].type == 3
        assert abs(values["srt"].double_value - 0.2) < 1e-6

        assert values["str"].type == 3
        assert abs(values["str"].double_value - 0.1) < 1e-6

        assert values["stt"].type == 3
        assert abs(values["stt"].double_value - 0.2) < 1e-6

        assert values["linearUpdate"].type == 3
        assert abs(values["linearUpdate"].double_value - 1.0) < 1e-6

        assert values["angularUpdate"].type == 3
        assert abs(values["angularUpdate"].double_value - 0.5) < 1e-6

        assert values["xmin"].type == 3
        assert abs(values["xmin"].double_value - (-100.0)) < 1e-6

        assert values["ymin"].type == 3
        assert abs(values["ymin"].double_value - (-100.0)) < 1e-6

        assert values["xmax"].type == 3
        assert abs(values["xmax"].double_value - 100.0) < 1e-6

        assert values["ymax"].type == 3
        assert abs(values["ymax"].double_value - 100.0) < 1e-6

        assert values["sigma"].type == 3
        assert abs(values["sigma"].double_value - 0.05) < 1e-6

        assert values["resampleThreshold"].type == 3
        assert abs(values["resampleThreshold"].double_value - 0.5) < 1e-6

        # Check integer params (type 2)
        assert values["particles"].type == 2
        assert values["particles"].integer_value == 30

        assert values["kernelSize"].type == 2
        assert values["kernelSize"].integer_value == 1

        assert values["iterations"].type == 2
        assert values["iterations"].integer_value == 5

        assert values["lskip"].type == 2
        assert values["lskip"].integer_value == 0

        assert values["throttle_scans"].type == 2
        assert values["throttle_scans"].integer_value == 1

    finally:
        if executor is not None:
            executor.shutdown()
        if test_node is not None:
            test_node.destroy_node()
        _kill_proc(proc)
        try:
            context.shutdown()
        except Exception:
            pass


def test_slam_gmapping_maxurange_capping():
    """Launch node with maxUrange > maxRange and verify capping."""
    domain_id = random.randint(50, 200)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    proc = None
    context = rclpy.Context()
    executor = None
    test_node = None
    try:
        context.init(args=[], domain_id=domain_id)

        proc = subprocess.Popen(
            [
                "ros2", "run", "task_013_slam_mapping_params", "slam_gmapping_node",
                "--ros-args",
                "-p", "maxRange:=10.0",
                "-p", "maxUrange:=15.0",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        time.sleep(3.0)

        test_node = Node("test_cap_client_urange", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(test_node)

        client = test_node.create_client(
            GetParameters, "/slam_gmapping/get_parameters"
        )

        result = _call_get_parameters(
            executor, test_node, client, ["maxUrange", "maxRange"]
        )

        max_urange_val = result.values[0].double_value
        max_range_val = result.values[1].double_value

        assert abs(max_range_val - 10.0) < 1e-6, \
            f"maxRange should be 10.0, got {max_range_val}"
        assert max_urange_val <= max_range_val + 1e-9, \
            f"maxUrange ({max_urange_val}) should be capped to maxRange ({max_range_val})"
        assert abs(max_urange_val - 10.0) < 1e-6, \
            f"maxUrange should be capped to 10.0, got {max_urange_val}"

    finally:
        if executor is not None:
            executor.shutdown()
        if test_node is not None:
            test_node.destroy_node()
        _kill_proc(proc)
        try:
            context.shutdown()
        except Exception:
            pass