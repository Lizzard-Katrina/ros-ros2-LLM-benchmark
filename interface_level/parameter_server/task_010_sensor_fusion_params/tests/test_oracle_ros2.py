import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "ros_filter.cpp"

def test_ros2_parameter_declaration():
    """Verify that ROS 2 declare_parameter is used with the correct template type."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Matches: this->declare_parameter<std::vector<bool>>
    pattern = r"this->declare_parameter\s*<\s*std::vector<bool>\s*>"
    assert re.search(pattern, content), "Failure: Missing mandatory ROS 2 'declare_parameter<std::vector<bool>>' call."

def test_sensor_config_logic():
    """Verify the naming logic for sensor config (e.g., odom0_config).
    Allows both string literals and variables as prefixes.
    """
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    
    # Matches: (variable or "literal") + (std::to_string or variable) + "_config"
    # This handles the LLM's optimized loop logic correctly.
    pattern = r"(?:\"[\w_]+\"|[\w_]+)\s*\+\s*(?:std::to_string\(.*\)|[\w_]+)\s*\+\s*\"_config\""
    assert re.search(pattern, content), "Failure: Could not find logic to construct parameter names like '<type><index>_config'."

def test_vector_15_validation():
    """Verify that the 15-element requirement for robot_localization is enforced."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Checks for the presence of '15' and a size check logic
    assert "15" in content and ".size()" in content, "Failure: Missing validation for the 15-element sensor configuration vector."

def test_no_ros1_nodehandle():
    """Ensure no legacy ROS 1 NodeHandle code exists."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    bad_patterns = [r"nh\.", r"ros::NodeHandle", r"\.getParam\("]
    for pattern in bad_patterns:
        assert not re.search(pattern, content), f"Failure: ROS 1 syntax detected: {pattern}"

def test_logger_migration():
    """Verify that ROS 2 RCLCPP logging is used with the node's logger."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Matches RCLCPP_WARN/INFO/ERROR(this->get_logger(), ...)
    pattern = r"RCLCPP_(?:INFO|WARN|ERROR)(?:_STREAM)?\s*\(\s*(?:this->)?get_logger\(\)"
    assert re.search(pattern, content), "Failure: Logging must use the node's logger (get_logger())."

def test_parameter_get_logic():
    """Verify that get_parameter is used to retrieve the configuration."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Ensures the code attempts to actually fetch the parameter after declaration
    assert "get_parameter" in content, "Failure: Missing 'get_parameter' call to retrieve values from the server."
