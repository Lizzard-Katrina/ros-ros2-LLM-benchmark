import re
import pytest
from pathlib import Path

# Setup paths for bridge_node task
BASE_DIR = Path(__file__).resolve().parents[1]
HPP_FILE = BASE_DIR / "bridge_node.hpp"
CPP_FILE = BASE_DIR / "bridge_node.cpp"

@pytest.fixture
def hpp_code():
    return HPP_FILE.read_text()

@pytest.fixture
def cpp_code():
    return CPP_FILE.read_text()

# --- Header Tests ---

def test_hpp_ros2_interface_migration(hpp_code):
    """Verify that ROS 1 Subscriber/Publisher are replaced with ROS 2 SharedPtrs."""
    # Pattern to match rclcpp SharedPtr interfaces
    pattern = r"std::vector<rclcpp::(?:Subscription|Publisher)(?:Base|Generic)?::SharedPtr>"
    assert re.search(pattern, hpp_code), "Header must declare ROS 2 SharedPtr interfaces."

def test_hpp_no_ros1_leakage(hpp_code):
    """Ensure no ros/ros.h or ros:: types remain in header."""
    assert "#include <ros/ros.h>" not in hpp_code
    assert "ros::Subscriber" not in hpp_code

# --- Source Tests ---

def test_cpp_node_inheritance_usage(cpp_code):
    """Check if the node uses 'this->' or a node pointer for ROS 2 API calls."""
    # ROS 2 usually uses this->create_publisher or node->create_publisher
    pattern = r"(?:this->|node->)create_(?:publisher|subscription)"
    assert re.search(pattern, cpp_code), "Source must use ROS 2 node creation patterns."

def test_cpp_ros2_serialization(cpp_code):
    """Verify migration of serialization logic to ROS 2 API."""
    # Look for rclcpp::Serialization or similar serialization patterns
    assert "rclcpp::Serialization" in cpp_code or "serialize" in cpp_code.lower()
    assert "ros::serialization" not in cpp_code, "Legacy ros::serialization detected in source."

def test_cpp_qos_usage(cpp_code):
    """Ensure QoS profiles are used in the bridge, which is critical for swarm stability."""
    # Pattern for QoS or SystemDefaultsQoS
    pattern = r"rclcpp::(?:QoS|SystemDefaultsQoS|SensorDataQoS)"
    assert re.search(pattern, cpp_code), "Migration should include ROS 2 QoS profiles."

def test_system_callback_consistency(hpp_code, cpp_code):
    """Check if the callback signature in CPP matches the SharedPtr type expected in ROS 2."""
    # ROS 2 callbacks use const T::SharedPtr msg or similar
    pattern = r"sub_cb\s*\(const\s+[\w:]+::SharedPtr\s+\w+\)"
    assert re.search(pattern, cpp_code), "Callback signature should use SharedPtr for ROS 2 compatibility."

def test_cpp_logging_migration(cpp_code):
    """Verify ROS_INFO/ERROR are migrated to RCLCPP_INFO/ERROR."""
    assert "RCLCPP_INFO" in cpp_code
    assert "ROS_INFO" not in cpp_code
def test_no_excessive_repetition(cpp_code):
    """Detect if the LLM is spamming includes to bypass logic generation."""
    lines = cpp_code.split('\n')
    unique_lines = set(lines)
    if len(lines) > 20 and len(unique_lines) / len(lines) < 0.5:
        assert False, "Detected code spamming (excessive repetitive lines)."
