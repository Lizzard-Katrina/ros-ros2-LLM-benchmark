"""
Tests for task_004_turtlrbot3_params.

Two layers:

1. Static oracle checks on the translated turtlebot3.cpp. The verbatim file
   cannot be compiled or run in this harness -- it depends on the turtlebot3_node
   package, DynamixelSDK and OpenCR hardware -- so structural / semantic
   correctness of the ROS1->ROS2 parameter-event migration is checked by regex,
   same as tests/test_oracle_ros2.py.

2. A real runtime check of the migrated parameter-event conversion pattern.
   ros2_code/source/src/profile_accel_param_node.cpp carries the *exact* migrated
   logic from turtlebot3.cpp (AsyncParametersClient + on_parameter_event +
   value / motors_.profile_acceleration_constant). These tests launch that node,
   change motors.profile_acceleration through the parameter service, and verify
   the callback actually runs and performs the division-based unit conversion.
"""

import re
import time
import subprocess
from pathlib import Path

import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType


# ─── Locate the translated source file ───
def _find_cpp_file():
    here = Path(__file__).resolve().parent
    for c in (here / "turtlebot3.cpp", here / "src" / "turtlebot3.cpp"):
        if c.exists():
            return c
    for parent in here.parents:
        for c in (parent / "turtlebot3.cpp", parent / "src" / "turtlebot3.cpp"):
            if c.exists():
                return c
    return here / "turtlebot3.cpp"


CPP_FILE = _find_cpp_file()
PROFILE_ACCEL_CONSTANT = 214.577


@pytest.fixture
def code_content():
    assert CPP_FILE.exists(), f"Cannot find turtlebot3.cpp at {CPP_FILE}"
    return CPP_FILE.read_text(encoding="utf-8")


# ─── Static oracle checks ───

def test_async_client_architecture(code_content):
    assert re.search(r"make_shared\s*<\s*(?:\w+::)?AsyncParametersClient\s*>", code_content), \
        "Must implement the Observer Pattern using AsyncParametersClient."


def test_service_readiness_logic(code_content):
    assert re.search(r"wait_for_service", code_content), \
        "Missing asynchronous service readiness check (wait_for_service)."


def test_api_constraint_compliance(code_content):
    assert "on_parameter_event" in code_content, "Must use 'on_parameter_event' for subscription."
    assert "add_on_set_parameters_callback" not in code_content, \
        "Constraint Violation: Used prohibited 'add_on_set_parameters_callback'."


def test_target_parameter_recognition(code_content):
    assert "motors.profile_acceleration" in code_content, \
        "Logic must target 'motors.profile_acceleration'."


def test_physics_logic_preservation(code_content):
    pattern = r"profile_acceleration\s*=\s*.*?\s*/\s*motors_\.profile_acceleration_constant"
    assert re.search(pattern, code_content), \
        "Physics Error: Acceleration should be DIVIDED by the constant for unit conversion."


def test_event_message_parsing(code_content):
    assert re.search(r"for\s*\(.*changed_parameters\)", code_content), \
        "Must correctly iterate through 'changed_parameters' in the event message."


def test_value_extraction_style(code_content):
    assert re.search(r"\.as_double\(|from_parameter_msg", code_content), \
        "Must use standard ROS2 methods to extract parameter values."


def test_logging_semantic_content(code_content):
    assert "rev/min2" in code_content, "Log message must contain the unit 'rev/min2'."


def test_no_legacy_ros1_symbols(code_content):
    for sym in ("ros::NodeHandle", "ros::ok", "getParam", "ros::param"):
        assert sym not in code_content, f"Legacy symbol '{sym}' found in migrated code."


# ─── Runtime checks against the executable reference node ───

NODE_NS = "/profile_accel_param_node"


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node_proc():
    proc = subprocess.Popen(
        ["ros2", "run", "task_004_turtlrbot3_params", "profile_accel_param_node"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    time.sleep(3.0)
    assert proc.poll() is None, "profile_accel_param_node exited prematurely"
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def _set_profile_acceleration(node, value, timeout=5.0):
    cli = node.create_client(SetParameters, f"{NODE_NS}/set_parameters")
    assert cli.wait_for_service(timeout_sec=timeout), "set_parameters service not available"
    req = SetParameters.Request()
    pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    req.parameters = [ParameterMsg(name="motors.profile_acceleration", value=pv)]
    fut = cli.call_async(req)
    end = time.time() + timeout
    while not fut.done() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert fut.done() and fut.result() is not None, "set_parameters call did not complete"
    assert fut.result().results[0].successful, fut.result().results[0].reason


def _await_converted(node, timeout=8.0):
    got = []
    sub = node.create_subscription(
        Float64, f"{NODE_NS}/profile_acceleration_converted", lambda m: got.append(m.data), 10)
    end = time.time() + timeout
    while not got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got


def test_parameter_event_triggers_conversion(ros, node_proc):
    """Changing motors.profile_acceleration must run the on_parameter_event
    callback and publish value / 214.577 (429.154 / 214.577 == 2.0)."""
    node = Node("test_pa_client")
    try:
        # subscribe first so the transient publish is not missed
        got = []
        sub = node.create_subscription(
            Float64, f"{NODE_NS}/profile_acceleration_converted", lambda m: got.append(m.data), 10)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        _set_profile_acceleration(node, 429.154)

        end = time.time() + 8.0
        while not got and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)

        assert got, "on_parameter_event callback produced no converted value"
        assert abs(got[-1] - 2.0) < 1e-3, f"expected 429.154/214.577 == 2.0, got {got[-1]}"
    finally:
        node.destroy_node()


def test_conversion_is_division_not_multiplication(ros, node_proc):
    node = Node("test_pa_div_client")
    try:
        got = []
        sub = node.create_subscription(
            Float64, f"{NODE_NS}/profile_acceleration_converted", lambda m: got.append(m.data), 10)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        raw = 1072.885  # 1072.885 / 214.577 == 5.0
        _set_profile_acceleration(node, raw)

        end = time.time() + 8.0
        while not got and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)

        assert got, "no converted value received"
        assert abs(got[-1] - 5.0) < 1e-3, f"division expected 5.0, got {got[-1]}"
        assert abs(got[-1] - raw * PROFILE_ACCEL_CONSTANT) > 1.0, \
            "value looks like multiplication, not division"
    finally:
        node.destroy_node()


def test_node_observes_parameter_events(ros, node_proc):
    """The migrated pattern uses AsyncParametersClient::on_parameter_event, i.e.
    the node subscribes to /parameter_events."""
    node = Node("test_pa_graph_client")
    try:
        deadline = time.time() + 15.0
        seen = False
        while time.time() < deadline and not seen:
            rclpy.spin_once(node, timeout_sec=0.2)
            if "profile_accel_param_node" not in node.get_node_names():
                continue
            try:
                subs = node.get_subscriber_names_and_types_by_node(
                    "profile_accel_param_node", "/")
            except Exception:
                continue
            if any(name == "/parameter_events" for name, _ in subs):
                seen = True
        assert seen, "node is not subscribed to /parameter_events (no on_parameter_event wiring)"
    finally:
        node.destroy_node()
