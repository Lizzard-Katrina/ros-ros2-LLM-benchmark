import re
import pytest
from pathlib import Path

# Path to the translated ROS2 code
CPP_FILE = Path(__file__).resolve().parents[1] / "turtlebot3.cpp"

@pytest.fixture
def code_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        # Clean comments to avoid false positives during regex matching
        content = f.read()
        return content
# --- 1. Infrastructure & Architecture Verification ---

def test_async_client_architecture(code_content):
    """
    Concept: ROS2 Asynchronous Observer Pattern.
    LLM must use AsyncParametersClient instead of simple callbacks as requested.
    """
    # Matches the creation of AsyncParametersClient
    pattern = r"make_shared\s*<\s*(?:\w+::)?AsyncParametersClient\s*>"
    assert re.search(pattern, code_content), "Must implement the Observer Pattern using AsyncParametersClient."

def test_service_readiness_logic(code_content):
    """
    Concept: Service-based parameter systems require waiting for the service to be ready.
    """
    # Matches wait_for_service call
    assert re.search(r"wait_for_service", code_content), "Missing asynchronous service readiness check (wait_for_service)."

def test_api_constraint_compliance(code_content):
    """
    Concept: Strict Instruction Following regarding the chosen API.
    Must use on_parameter_event and NOT add_on_set_parameters_callback.
    """
    assert "on_parameter_event" in code_content, "Must use 'on_parameter_event' for subscription."
    assert "add_on_set_parameters_callback" not in code_content, "Constraint Violation: Used prohibited 'add_on_set_parameters_callback'."


# --- 2. Semantic Logic & Physics Verification ---

def test_target_parameter_recognition(code_content):
    """
    Concept: Identifying the correct parameter string from the logic requirements.
    """
    # Matches the specific parameter name
    assert "motors.profile_acceleration" in code_content, "Logic must target 'motors.profile_acceleration'."

def test_physics_logic_preservation(code_content):
    """
    Concept: Mathematical correctness in physical unit conversion.
    The LLM must deduce the 'division' from the migration context.
    """
    # Look for the pattern where acceleration is divided by the constant
    # This catches the mistake where the model might use multiplication (*)
    pattern = r"profile_acceleration\s*=\s*.*?\s*/\s*motors_\.profile_acceleration_constant"
    assert re.search(pattern, code_content), "Physics Error: Acceleration should be DIVIDED by the constant for unit conversion."

def test_event_message_parsing(code_content):
    """
    Concept: Correct traversal of the ROS2 ParameterEvent message structure.
    """
    # Logic: Must iterate through changed_parameters (the standard structure of ParameterEvent)
    pattern = r"for\s*\(.*changed_parameters\)"
    assert re.search(pattern, code_content), "Must correctly iterate through 'changed_parameters' in the event message."

def test_value_extraction_style(code_content):
    """
    Concept: Using rclcpp standard value extraction.
    """
    # Matches .as_double() or conversion from the parameter message
    assert re.search(r"\.as_double\(|from_parameter_msg", code_content), "Must use standard ROS2 methods to extract parameter values."


# --- 3. Cleanliness & Logging ---

def test_logging_semantic_content(code_content):
    """
    Concept: Verifying the log contains the specific unit 'rev/min2' as a feedback requirement.
    """
    assert "rev/min2" in code_content, "Log message must contain the unit 'rev/min2'."

def test_no_legacy_ros1_symbols(code_content):
    """
    Concept: General migration quality - no ROS1 leakage.
    """
    legacy = ["ros::NodeHandle", "ros::ok", "getParam", "ros::param"]
    for sym in legacy:
        assert sym not in code_content, f"Legacy symbol '{sym}' found in migrated code."
