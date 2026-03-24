import re
from pathlib import Path

# Path to the source file under test
CPP_FILE = Path(__file__).resolve().parents[1] /"slam_gmapping.cpp"

def get_content():
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Remove comments to avoid false positives from TODO descriptions
    content = re.sub(r'//.*|/\*[\s\S]*?\*/', '', content)
    return content

def test_ros2_lifecycle_sequence():
    """Concept: Ensure parameters are declared before they are accessed."""
    content = get_content()
    # Must use the Node's native declare_parameter method
    assert "this->declare_parameter" in content or "declare_parameter" in content, \
        "Failure: ROS 2 parameters must be explicitly declared."
    # Ensure they don't just declare but actually 'get' the values
    assert "get_parameter" in content, "Failure: Declared parameters must be retrieved."

def test_scanner_physical_validation_logic():
    """Concept: Autonomous implementation of 'maxUrange <= maxRange' safety check."""
    content = get_content()
    # Check for logical comparison between the two range parameters
    validation_pattern = r"if\s*\(\s*\w*maxUrange\w*\s*>\s*\w*maxRange\w*\s*\)"
    assert re.search(validation_pattern, content), \
        "Failure: Missing physical consistency check. maxUrange must be capped by maxRange."

def test_explicit_type_casting():
    """Concept: Use of explicit getters like .as_double() to ensure type safety."""
    content = get_content()
    # ROS 2 best practice: avoid ambiguous overloads, use explicit casting
    # This prevents the LLM from using ROS 1-style 'get_parameter("name", var)'
    assert re.search(r"\.as_(?:double|int|string|bool)\(\)", content), \
        "Failure: Use explicit type casting (e.g., .as_double()) for ROS 2 parameters."

def test_logging_api_migration():
    """Concept: Complete migration to RCLCPP logging macros."""
    content = get_content()
    # Ensure no ROS 1 macros remain
    assert not re.search(r"ROS_(?:INFO|WARN|ERROR|DEBUG)", content), \
        "Failure: Legacy ROS 1 logging macros detected. Use RCLCPP_WARN or similar."
    # Ensure get_logger() is used with the node's logger
    assert "get_logger()" in content or "RCLCPP_" in content, \
        "Failure: ROS 2 logging API not found."

def test_no_fake_nodehandle():
    """Concept: Reject the 'private_nh_' variable name in ROS 2."""
    content = get_content()
    # The previous test failed because the LLM named a variable 'private_nh_' 
    # to mimic ROS 1. In ROS 2, parameters belong to the Node (this->).
    assert "private_nh_" not in content, \
        "Failure: 'private_nh_' detected. In ROS 2, use 'this->declare_parameter' directly."

def test_slam_default_values():
    """Concept: Check if critical SLAM-specific defaults are preserved."""
    content = get_content()
    # temporalUpdate is often -1.0 in GMapping to disable time-based updates
    assert re.search(r"temporalUpdate['\"].*?-1\.0", content) or "temporalUpdate" in content, \
        "Failure: temporalUpdate parameter logic is missing or incorrect."
    # Check for map resolution (delta)
    assert "0.05" in content and "delta" in content, \
        "Failure: Map resolution (delta) default of 0.05 not found."

def test_member_variable_assignment():
    """Concept: Ensure parameters are stored in the correct class members."""
    content = get_content()
    # Look for assignments to member variables (usually ending in _)
    # Example: maxUrange_ = this->get_parameter(...).as_double();
    member_assignment = r"\w+_\s*=\s*(?:this->)?get_parameter\(.*?\)\.as_"
    assert re.search(member_assignment, content), \
        "Failure: Parameters must be explicitly assigned to class members using .as_type() syntax."
