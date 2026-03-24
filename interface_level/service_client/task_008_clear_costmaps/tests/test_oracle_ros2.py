# src/task_008/test/test_oracle_ros2.py
import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "clear_tester.cpp"


# ----------------------------
# Helpers
# ----------------------------

def read_file(path: Path) -> str:
    if not path.exists():
        raise AssertionError(f"Expected C++ file not found: {path}")
    return path.read_text(encoding="utf-8", errors="ignore")


def _re_search(pattern: str, text: str, flags=re.MULTILINE | re.DOTALL) -> bool:
    try:
        return re.search(pattern, text, flags) is not None
    except re.error as e:
        raise AssertionError(f"Oracle regex invalid:\n{pattern}\nRegex error: {e}")


def assert_has(pattern: str, text: str, msg: str):
    if not _re_search(pattern, text):
        raise AssertionError(msg + f"\nMissing pattern: {pattern}\nFile: {CPP_FILE}")


def assert_not_has(pattern: str, text: str, msg: str):
    if _re_search(pattern, text):
        raise AssertionError(msg + f"\nFound forbidden pattern: {pattern}\nFile: {CPP_FILE}")


def extract_function_body(code: str, func_name: str) -> str:
    """
    Best-effort extraction of a C++ function body by name.
    Robust enough for oracle use; not a full C++ parser.
    """
    m = re.search(rf'\b{re.escape(func_name)}\s*\([^)]*\)\s*\{{', code, re.MULTILINE | re.DOTALL)
    if not m:
        return ""
    i = m.end()  # position after the opening "{"
    depth = 1
    while i < len(code) and depth > 0:
        if code[i] == "{":
            depth += 1
        elif code[i] == "}":
            depth -= 1
        i += 1
    return code[m.start():i] if depth == 0 else ""


# ----------------------------
# Oracle tests (equivalence-focused)
# ----------------------------

def test_no_ros1_core_api_remnants():
    """
    Equivalence guard: should be ROS2, no ROS1 runtime API.
    (We do NOT ban boost::shared_ptr because ROS2 deps may still use it.)
    """
    code = read_file(CPP_FILE)
    assert_has(r'#include\s*<\s*rclcpp/[^>]+>', code, "Expected ROS2 include: #include <rclcpp/...>.")
    assert_not_has(r'#include\s*<\s*ros/ros\.h\s*>', code, "ROS1 header <ros/ros.h> must not appear.")
    for pat, msg in [
        (r'\bros::NodeHandle\b', "ROS1 ros::NodeHandle must not appear."),
        (r'\bros::init\s*\(', "ROS1 ros::init must not appear (use rclcpp::init)."),
        (r'\bros::spin\s*\(', "ROS1 ros::spin must not appear (use rclcpp::spin/spin_some/...)."),
        (r'\bROS_(INFO|WARN|ERROR|DEBUG|FATAL)\b', "ROS1 ROS_* logging macros must not appear."),
    ]:
        assert_not_has(pat, code, msg)


def test_testCountLethal_calls_testClearBehavior():
    """
    Equivalence: keep the same test structure - testCountLethal triggers the clear behavior via testClearBehavior.
    """
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "testCountLethal")
    if not body:
        raise AssertionError("Expected function testCountLethal(...) to exist with a body.")
    assert_has(
        r'\btestClearBehavior\s*\(',
        body,
        "testCountLethal(...) should call testClearBehavior(...) to trigger the clear behavior.",
    )


def test_testClearBehavior_exists_and_not_stub():
    """
    The 'hole' must be filled: testClearBehavior exists and does real work.
    """
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "testClearBehavior")
    if not body:
        raise AssertionError("Expected function testClearBehavior(...) to exist with a non-empty body.")

    # Must do at least one meaningful ROS2 action: parameters or service client.
    if not _re_search(r'(declare_parameter\s*\(|set_parameter\s*\(|set_parameters\s*\(|create_client\s*<|async_send_request\s*\(|send_request\s*\()', body):
        raise AssertionError(
            "testClearBehavior(...) looks like a stub.\n"
            "Expected it to set parameters and/or call a service client."
        )


def test_equivalence_reset_distance_is_expressed():
    """
    Must preserve the reset_distance idea from ROS1:
      - either a ROS2 parameter with key 'reset_distance', OR
      - a request field assignment that clearly sets a distance/radius.
    """
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "testClearBehavior")
    if not body:
        raise AssertionError("Missing testClearBehavior body.")

    has_param_key = _re_search(r'["\']reset_distance["\']', body)
    uses_param_api = _re_search(r'(declare_parameter\s*\(|set_parameter\s*\(|set_parameters\s*\()', body)

    # accept common names: reset_distance / distance / radius
    has_req_assign = _re_search(
        r'(request|req)\s*->\s*(reset_distance|distance|radius)\s*=\s*',
        body,
    )

    if not ((has_param_key and uses_param_api) or has_req_assign):
        raise AssertionError(
            "Missing equivalence for reset_distance.\n"
            "Expected either:\n"
            "- ROS2 parameter usage with key \"reset_distance\" (declare/set), OR\n"
            "- request field assignment like req->distance = ... / req->reset_distance = ... / req->radius = ...\n"
            f"File: {CPP_FILE}"
        )


def test_equivalence_layer_names_semantics_obstacles_static_map():
    """
    Must preserve layer_names logic:
    - The strings 'obstacles' and/or 'static_map' appear, AND
    - They are gated by the function arguments obstacles/static_map (or equivalent conditionals).
    - Accept either parameter key 'layer_names' OR request field assignment for layers.
    """
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "testClearBehavior")
    if not body:
        raise AssertionError("Missing testClearBehavior body.")

    # Must mention the relevant layer names somewhere
    has_obstacles_literal = _re_search(r'["\']obstacles["\']', body)
    has_static_literal = _re_search(r'["\']static_map["\']', body)
    if not (has_obstacles_literal or has_static_literal):
        raise AssertionError(
            "Missing layer name literals.\n"
            "Expected at least one of: \"obstacles\" or \"static_map\" to appear in testClearBehavior."
        )

    # Must be conditional on input flags (robust, allows spaces, braces, etc.)
    # We only require at least one such gate; many correct solutions will have both.
    gated_by_flags = (
        _re_search(r'\bif\s*\(\s*obstacles\s*\)', body) or
        _re_search(r'\bif\s*\(\s*static_map\s*\)', body) or
        _re_search(r'\bobstacles\s*\?\s*', body) or
        _re_search(r'\bstatic_map\s*\?\s*', body)
    )
    if not gated_by_flags:
        raise Asse

def test_equivalence_gates_both_obstacles_and_static_map():
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "testClearBehavior")
    if not body:
        raise AssertionError("Missing testClearBehavior body.")

    gate_obs = _re_search(r'(\bif\s*\(\s*obstacles\s*\)|\bobstacles\s*\?)', body)
    gate_static = _re_search(r'(\bif\s*\(\s*static_map\s*\)|\bstatic_map\s*\?)', body)

    if not (gate_obs and gate_static):
        raise AssertionError(
            "Expected layer selection logic to consider BOTH flags: obstacles and static_map.\n"
            "In ROS1, both independently contribute to layer_names; missing one changes behavior."
        )


