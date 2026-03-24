import re
import pytest
from pathlib import Path

# Path to the code translated by LLM
CPP_FILE = Path(__file__).resolve().parents[1] / "diff_drive_controller.cpp"

@pytest.fixture
def code_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        return f.read()

# --- Task 1 & 2: Instruction Following & Style ---

def test_ros2_logging_style_compliance(code_content):
    """
    STRICT CONSTRAINT: Must use RCLCPP_INFO_STREAM.
    If LLM uses RCLCPP_INFO, this test MUST fail.
    """
    # Specifically looking for the _STREAM suffix as requested in TODO
    pattern = r"RCLCPP_INFO_STREAM\s*\(\s*\w+->get_logger\(\)"
    assert re.search(pattern, code_content), "LLM failed to follow the style constraint: Use RCLCPP_INFO_STREAM."

def test_cmd_vel_timeout_assignment(code_content):
    """
    CONCEPT: cmd_vel_timeout_ must be assigned from a double parameter.
    """
    pattern = r"cmd_vel_timeout_\s*=\s*.*as_double\(\)"
    assert re.search(pattern, code_content), "cmd_vel_timeout_ was not properly assigned as a double."


# --- Task 3: Exact String & Logic Match ---

def test_exception_message_exact_match(code_content):
    """
    STRICT CONSTRAINT: Throw with message 'diagonal size must be 6'.
    If LLM adds prefixes (like variable names), it fails this strict match.
    """
    # We use escaped quotes to ensure the string is exactly what was requested
    expected_msg = r'\"diagonal size must be 6\"'
    pattern = r"throw\s+std::invalid_argument\s*\(\s*" + expected_msg + r"\s*\)"
    assert re.search(pattern, code_content), "LLM failed to use the EXACT exception message specified in TODO."

def test_covariance_vector_type(code_content):
    """
    CONCEPT: Use std::vector<double> as explicitly requested in Style Constraints.
    """
    pattern = r"declare_parameter\s*<\s*std::vector\s*<\s*double\s*>\s*>"
    assert re.search(pattern, code_content), "LLM failed to use std::vector<double> for covariance diagonals."

def test_size_logic_check(code_content):
    """
    LOGIC: Verify the size check (size != 6) is present.
    """
    pattern = r"\.size\(\s*\)\s*!=\s*6"
    assert re.search(pattern, code_content), "Missing size != 6 validation logic."


# --- Cleanliness & Migration Sanity ---

def test_absence_of_xmlrpc(code_content):
    """
    SANITY: Ensure no ROS1 XmlRpc logic is remaining.
    """
    assert "XmlRpc" not in code_content, "Found legacy ROS1 XmlRpc symbols."

def test_no_ros1_nodehandle(code_content):
    """
    SANITY: Ensure ros::NodeHandle is completely replaced.
    """
    assert "ros::NodeHandle" not in code_content, "Found legacy ros::NodeHandle."
