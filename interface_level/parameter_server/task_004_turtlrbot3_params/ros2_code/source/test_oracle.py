import re
import pytest
from pathlib import Path

# Path to the translated ROS2 code
CPP_FILE = Path(__file__).resolve().parent / "turtlebot3.cpp"

@pytest.fixture
def code_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        content = f.read()
        return content

# --- 1. Infrastructure & Architecture Verification ---

def test_async_client_architecture(code_content):
    pattern = r"make_shared\s*<\s*(?:\w+::)?AsyncParametersClient\s*>"
    assert re.search(pattern, code_content), "Must implement the Observer Pattern using AsyncParametersClient."

def test_service_readiness_logic(code_content):
    assert re.search(r"wait_for_service", code_content), "Missing asynchronous service readiness check (wait_for_service)."

def test_api_constraint_compliance(code_content):
    assert "on_parameter_event" in code_content, "Must use 'on_parameter_event' for subscription."
    assert "add_on_set_parameters_callback" not in code_content, "Constraint Violation: Used prohibited 'add_on_set_parameters_callback'."

# --- 2. Semantic Logic & Physics Verification ---

def test_target_parameter_recognition(code_content):
    assert "motors.profile_acceleration" in code_content, "Logic must target 'motors.profile_acceleration'."

def test_physics_logic_preservation(code_content):
    pattern = r"profile_acceleration\s*=\s*.*?\s*/\s*motors_\.profile_acceleration_constant"
    assert re.search(pattern, code_content), "Physics Error: Acceleration should be DIVIDED by the constant for unit conversion."

def test_event_message_parsing(code_content):
    pattern = r"for\s*\(.*changed_parameters\)"
    assert re.search(pattern, code_content), "Must correctly iterate through 'changed_parameters' in the event message."

def test_value_extraction_style(code_content):
    assert re.search(r"\.as_double\(|from_parameter_msg", code_content), "Must use standard ROS2 methods to extract parameter values."

# --- 3. Cleanliness & Logging ---

def test_logging_semantic_content(code_content):
    assert "rev/min2" in code_content, "Log message must contain the unit 'rev/min2'."

def test_no_legacy_ros1_symbols(code_content):
    legacy = ["ros::NodeHandle", "ros::ok", "getParam", "ros::param"]
    for sym in legacy:
        assert sym not in code_content, f"Legacy symbol '{sym}' found in migrated code."