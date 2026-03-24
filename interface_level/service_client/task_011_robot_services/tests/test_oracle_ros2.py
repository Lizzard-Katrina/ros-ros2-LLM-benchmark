import re
from pathlib import Path

CLIENT_FILE =  Path(__file__).resolve().parents[1] /"camera_reconfigure.py"


def read_file(p: Path) -> str:
    assert p.exists(), f"Missing file: {p}"
    return p.read_text(encoding="utf-8", errors="ignore")


def _extract_function_block(code: str, func_name: str) -> str:
    m = re.search(rf"(?m)^(\s*)def\s+{re.escape(func_name)}\s*\(.*\)\s*:\s*$", code)
    assert m, f"Function '{func_name}' not found"
    indent = m.group(1)
    start = m.start()
    # next def/class at same or less indentation
    m2 = re.search(rf"(?m)^(?:{indent}def\s+\w+|{indent}class\s+\w+)\b", code[m.end():])
    end = (m.end() + m2.start()) if m2 else len(code)
    return code[start:end]


def test_ros2_only_no_rospy_dynamic_reconfigure():
    code = read_file(CLIENT_FILE)
    assert ("import rclpy" in code) or ("from rclpy" in code), "Must use ROS2 rclpy"
    assert "import rospy" not in code and "rospy." not in code, "Must not use rospy in ROS2 solution"
    assert "dynamic_reconfigure" not in code, "ROS2 solution must not depend on dynamic_reconfigure"


def test_target_camera_driver_is_referenced():
    code = read_file(CLIENT_FILE)
    # We don't hardcode exact API, but require the semantic target to appear.
    assert re.search(r"head_camera\s*/\s*driver|head_camera/driver", code), \
        "Must reference the target camera driver namespace 'head_camera/driver' (semantic target preservation)"


def test_parameters_are_used():
    code = read_file(CLIENT_FILE)
    # Accept either direct set_parameters or parameter client/service usage
    assert re.search(r"set_parameters\s*\(|SetParameters|parameter", code), \
        "Must use ROS2 parameters mechanism (set_parameters or parameter client/service)"


def test_init_sets_up_configuration_interface():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "__init__")
    # Should create a node or hold a node reference
    assert re.search(r"create_node\s*\(|Node\(", blk), \
        "__init__ should initialize a ROS2 node or hold a node reference"
    # Should prepare a way to talk to the camera driver node (parameter client/service, etc.)
    assert re.search(r"parameter|set_parameters|create_client|AsyncParametersClient", blk), \
        "__init__ should set up a configuration interface to the target node"


def test_disable_auto_sets_both_flags_false():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "disable_auto")
    # Must mention both parameter keys
    assert "auto_exposure" in blk and "auto_white_balance" in blk, \
        "disable_auto must set both auto_exposure and auto_white_balance"
    # Must set to False in this function
    assert re.search(r"auto_exposure.*False|False.*auto_exposure", blk), \
        "disable_auto must set auto_exposure to False"
    assert re.search(r"auto_white_balance.*False|False.*auto_white_balance", blk), \
        "disable_auto must set auto_white_balance to False"


def test_enable_auto_sets_both_flags_true():
    code = read_file(CLIENT_FILE)
    blk = _extract_function_block(code, "enable_auto")
    # Must mention both parameter keys
    assert "auto_exposure" in blk and "auto_white_balance" in blk, \
        "enable_auto must set both auto_exposure and auto_white_balance"
    # Must set to True in this function
    assert re.search(r"auto_exposure.*True|True.*auto_exposure", blk), \
        "enable_auto must set auto_exposure to True"
    assert re.search(r"auto_white_balance.*True|True.*auto_white_balance", blk), \
        "enable_auto must set auto_white_balance to True"
