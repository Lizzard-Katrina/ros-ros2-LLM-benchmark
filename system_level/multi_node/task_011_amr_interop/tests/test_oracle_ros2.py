import re
import pytest
from pathlib import Path

# --- Configuration: Paths to the generated/hollowed files ---
# In a real evaluation environment, these would point to the model's output
BASE_DIR = Path(__file__).resolve().parents[1]
CPP_HEADER = BASE_DIR / "state_handler.hpp"
PY_BRIDGE = BASE_DIR / "mqtt_bridge.py"

def get_content(file_path):
    if not file_path.exists():
        return ""
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# --- C++ INTERFACE TESTS ---

def test_cpp_state_handler_interface():
    """
    Validates that StateHandler is a correct abstract interface 
    inherited from Handler.
    """
    content = get_content(CPP_HEADER)
    
    # 1. Check Inheritance
    inheritance_pattern = r"class\s+StateHandler\s*:\s*public\s+Handler"
    assert re.search(inheritance_pattern, content), \
        "StateHandler must inherit publicly from Handler."

    # 2. Check Pure Virtual Methods (Interface Contract)
    assert re.search(r"virtual\s+void\s+configure\(\s*\)\s*=\s*0\s*;", content), \
        "configure() must be a pure virtual method."
    assert re.search(r"virtual\s+void\s+execute\(\s*\)\s*=\s*0\s*;", content), \
        "execute() must be a pure virtual method."

    # 3. Check Namespace
    assert "namespace adapter" in content, "Code must be inside 'adapter' namespace."


# --- PYTHON INTEROPERABILITY TESTS ---

def test_py_vda_protocol_evolution_logic():
    """
    Validates the 'HACK' for VDA5050 v1 vs v2 compatibility in instant actions.
    """
    content = get_content(PY_BRIDGE)
    
    # 1. Detection of v1 field
    assert "instant_actions" in content, \
        "The logic must check for the legacy 'instant_actions' field (v1)."
    
    # 2. Mapping to v2 field
    assert "actions" in content, \
        "The logic must map to the 'actions' field (v2)."
    
    # 3. Parameter Type Safety (The string cast requirement)
    # Looking for: action_parameter["value"] = str(...)
    type_cast_pattern = r"str\(.*\[[\"']value[\"']\]\)"
    assert re.search(type_cast_pattern, content), \
        "Action parameters must be cast to string to satisfy VDAActionParameter requirements."


def test_py_ros_to_mqtt_orchestration():
    """
    Validates that ROS subscriptions are correctly wired to MQTT publishers.
    """
    content = get_content(PY_BRIDGE)
    
    # 1. Helper usage (Ensures system tools are used instead of hardcoding)
    assert "get_vda5050_ros2_topic" in content, \
        "Must use 'get_vda5050_ros2_topic' helper for topic generation."

    # 2. Critical Mappings (Message Type -> Callback)
    # Check if create_subscription is called with the right msg_type and callback
    mapping_checks = [
        ("VDAOrderState", "_publish_state"),
        ("VDAConnection", "_publish_connection"),
        ("VDAVisualization", "_publish_visualization")
    ]
    
    for msg, callback in mapping_checks:
        # Regex to find subscription calls that link the message type to the callback
        pattern = rf"msg_type\s*=\s*{msg}.*callback\s*=\s*self\.{callback}"
        assert re.search(pattern, content, re.DOTALL), \
            f"Subscription for {msg} must be linked to callback self.{callback}."


def test_py_import_integrity():
    """
    Ensures the model didn't break existing imports required for the bridge.
    """
    content = get_content(PY_BRIDGE)
    required_imports = [
        "from vda5050_msgs.msg import OrderState as VDAOrderState",
        "from vda5050_connector_py.utils import get_vda5050_ros2_topic"
    ]
    for imp in required_imports:
        assert imp in content, f"Missing required import: {imp}"

# --- SYSTEM-LEVEL COUPLING TEST ---

def test_cross_file_naming_consistency():
    """
    Check if the Python callback names match the intended C++ handler logic 
    implied by the system design.
    """
    py_content = get_content(PY_BRIDGE)
    # The Python bridge expects to publish 'state'
    # This proves the model understands that StateHandler (C++) output
    # is consumed by _publish_state (Python)
    assert "_publish_state" in py_content and "topic" in py_content and "state" in py_content, \
        "System coupling failed: Python bridge does not have a handler for 'state' updates."


def test_no_ros1_terminology_leakage():
    """
    Ensure no legacy ROS 1 Python API remains. 
    In ROS 2, 'rospy' is replaced by 'rclpy'.
    """
    py_content = get_content(PY_BRIDGE)
    
    legacy_terms = [
        "rospy", 
        "roslib", 
        "Publisher(",  # ROS 1 style: rospy.Publisher
        "Subscriber(", # ROS 1 style: rospy.Subscriber
        "get_param",   # ROS 2 uses get_parameter
        "Time.now()"   # ROS 2 uses self.get_clock().now()
    ]
    
    for term in legacy_terms:
        pattern = rf"\b{term}\b"
        assert not re.search(pattern, py_content), \
            f"Legacy ROS 1 term '{term}' found. This is a ROS 2 task!"
