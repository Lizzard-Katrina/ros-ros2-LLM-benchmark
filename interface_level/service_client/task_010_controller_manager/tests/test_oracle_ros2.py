import re
from pathlib import Path

CLIENT_FILE=Path(__file__).resolve().parents[1] / "controller_manager_interface.py"

def read_file(p: Path) -> str:
    assert p.exists(), f"Missing file: {p}"
    return p.read_text(encoding="utf-8", errors="ignore")


def _extract_function_block(code: str, func_name: str) -> str:
    """
    Very lightweight extraction: grabs 'def func_name(...):' up to next 'def ' at same indentation.
    Works well enough for oracle checks in these tasks.
    """
    m = re.search(rf"(?m)^def\s+{re.escape(func_name)}\s*\(.*\)\s*:\s*$", code)
    assert m, f"Function '{func_name}' not found"
    start = m.start()
    # find next top-level def
    m2 = re.search(r"(?m)^def\s+\w+\s*\(.*\)\s*:\s*$", code[m.end():])
    end = (m.end() + m2.start()) if m2 else len(code)
    return code[start:end]



def _has_ros2_async_service_call(block: str) -> bool:
    """
    rclpy idioms vary. Accept either:
      - client.call_async(req)
      - client.async_send_request(req)   (if a wrapper exists)
    """
    return re.search(r"(call_async|async_send_request)\s*\(", block) is not None


def _has_wait_for_future(block: str) -> bool:
    """
    Must wait for service future completion somehow (most typical):
      - rclpy.spin_until_future_complete(node, future)
    We allow any 'spin_until_future_complete(' usage.
    """
    return re.search(r"spin_until_future_complete\s*\(", block) is not None


def _return_depends_on_response(block: str) -> bool:
    """
    Return must depend on response.success/ok (not constant).
    Accept response variable names like resp/response/result.
    """
    return re.search(r"\b(resp|response|result)\b\s*\.\s*(success|ok)\b", block) is not None


def _disallow_constant_boolean_return(block: str) -> None:
    assert not re.search(r"(?m)^\s*return\s+(True|False)\s*$", block), \
        "Do not return a constant True/False; return must reflect the service response."


# ----------------------------
# Global sanity checks
# ----------------------------

def test_is_ros2_python_not_rospy():
    code = read_file(CLIENT_FILE)
    assert ("import rclpy" in code) or ("from rclpy" in code), "ROS2 Python client must use rclpy"
    assert "import rospy" not in code and "rospy." not in code, "ROS2 client should not use rospy"


def test_uses_service_clients():
    code = read_file(CLIENT_FILE)
    # Must create clients somewhere in file
    assert re.search(r"\bcreate_client\s*\(", code), "Must create ROS2 service clients via node.create_client(...)"
    # Must have at least one async call somewhere
    assert re.search(r"(call_async|async_send_request)\s*\(", code), "Must perform async service calls (call_async/async_send_request)"


# ----------------------------
# Function-level semantic oracles
# ----------------------------

def test_start_stop_controllers_semantics():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "start_stop_controllers")

    # 1) Must call switch controller service (type-level, not string-level)
    assert re.search(r"(SwitchController)\b", blk), \
        "start_stop_controllers must use the SwitchController service type"

    # 2) Must send an async request and wait for completion
    assert _has_ros2_async_service_call(blk), \
        "start_stop_controllers must send an async service request (call_async/async_send_request)"
    assert _has_wait_for_future(blk), \
        "start_stop_controllers must wait for the service future (spin_until_future_complete)"

    # 3) Request must reflect both start and stop lists (semantic mapping)
    # We avoid over-prescribing attribute names, but require both signals appear in request construction area.
    assert re.search(r"start_controllers", blk), \
        "start_stop_controllers must pass through the start_controllers argument into the request"
    assert re.search(r"stop_controllers", blk), \
        "start_stop_controllers must pass through the stop_controllers argument into the request"

    # 4) Return must depend on response (ok/success), not constant
    assert _return_depends_on_response(blk), \
        "start_stop_controllers return must depend on resp.ok/resp.success"
    _disallow_constant_boolean_return(blk)


def test_list_controllers_semantics():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "list_controllers")

    # 1) Must call list controllers service type
    assert re.search(r"(ListControllers)\b", blk), \
        "list_controllers must use the ListControllers service type"

    # 2) Must send async request + wait
    assert _has_ros2_async_service_call(blk), \
        "list_controllers must send an async service request (call_async/async_send_request)"
    assert _has_wait_for_future(blk), \
        "list_controllers must wait for the service future (spin_until_future_complete)"

    # 3) Must iterate response.controller (core semantic output)
    assert re.search(r"for\s+\w+\s+in\s+.*controller", blk), \
        "list_controllers should iterate over response.controller"

    # 4) Should summarize claimed resources / hardware interfaces (semantic preservation)
    # We keep this lenient: require the concept of claimed resources appears.
    assert re.search(r"claimed", blk, re.IGNORECASE), \
        "list_controllers should process claimed resources / interfaces to summarize output"

    # 5) Must print / log something (it is a CLI-style utility in ROS1)
    assert ("print(" in blk) or re.search(r"\bget_logger\s*\(", blk), \
        "list_controllers should output a summary (print or node.get_logger())"


def test_reload_libraries_semantics():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "reload_libraries")

    # 1) Must use ReloadControllerLibraries service type
    assert re.search(r"(ReloadControllerLibraries)\b", blk), \
        "reload_libraries must use the ReloadControllerLibraries service type"

    # 2) Must have restore branch (ROS1 behavior)
    assert re.search(r"\bif\s+restore\b", blk), \
        "reload_libraries must include an optional restore path when restore=True"

    # 3) Must include semantic tools used in restore: list/load/switch service types
    # (type-level checks; no need to match variable names or English words)
    assert re.search(r"(ListControllers)\b", blk), \
        "reload_libraries should include ListControllers usage (snapshot before restore)"
    assert re.search(r"(LoadController)\b", blk), \
        "reload_libraries should include LoadController usage (reload then re-load controllers)"
    assert re.search(r"(SwitchController)\b", blk), \
        "reload_libraries should include SwitchController usage (restart controllers that were running)"

    # 4) Must send async request(s) and wait at least once (reload call itself)
    assert _has_ros2_async_service_call(blk), \
        "reload_libraries must send async service request(s) (call_async/async_send_request)"
    assert _has_wait_for_future(blk), \
        "reload_libraries must wait for service future(s) (spin_until_future_complete)"

    # 5) Must pass force_kill into reload request (semantic parameter preservation)
    assert re.search(r"force_kill", blk), \
        "reload_libraries must pass through the force_kill argument into the reload request"

    # 6) Return must depend on reload response (ok/success), not constant
    assert _return_depends_on_response(blk), \
        "reload_libraries return must depend on resp.ok/resp.success"
    _disallow_constant_boolean_return(blk)
