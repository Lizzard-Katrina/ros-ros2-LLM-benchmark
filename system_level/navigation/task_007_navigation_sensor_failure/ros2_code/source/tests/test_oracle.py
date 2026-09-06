import re
from pathlib import Path
import pytest

# Paths should be adjusted based on your benchmark directory structure
HPP_FILE = Path(__file__).resolve().parents[1] / "turtle_frame.hpp"
CPP_FILE = Path(__file__).resolve().parents[1] / "turtle_frame.cpp"

def get_content(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

## --- Oracle Tests ---

def test_executor_and_node_declaration():
    """Verify system-level members are correctly declared in the header."""
    content = get_content(HPP_FILE)
    assert re.search(r"rclcpp::Node::SharedPtr\s+nh_;", content)
    assert re.search(r"rclcpp::executors::SingleThreadedExecutor\s+executor_;", content)

def test_service_callback_signatures():
    """Verify cross-file consistency of service callback signatures."""
    content = get_content(HPP_FILE)
    pattern = r"bool\s+\w+Callback\s*\(\s*const\s+.*::Request::SharedPtr\s*,\s*.*::Response::SharedPtr\s*\)"
    assert re.search(pattern, content), "Header callbacks must use (Req::SharedPtr, Res::SharedPtr)."

def test_executor_node_linkage():
    """Verify that the node is actually attached to the executor in the source."""
    content = get_content(CPP_FILE)
    assert re.search(r"executor_\.add_node\s*\(\s*nh_\s*\)", content), "Node not added to executor."

def test_parameter_safety_logic():
    """Verify mandatory ROS 2 parameter declaration with descriptors."""
    content = get_content(CPP_FILE)
    assert "integer_range.push_back" in content
    assert "nh_->declare_parameter" in content

def test_non_blocking_execution_logic():
    """Verify the update loop uses non-blocking spin to maintain UI responsiveness."""
    content = get_content(CPP_FILE)
    assert re.search(r"executor_\.spin_some\s*\(\s*\)", content)

def test_service_binding_logic():
    """Verify that create_service uses the correct bind pattern with two placeholders."""
    content = get_content(CPP_FILE)
    pattern = r"create_service<.*>\s*\(.*std::bind\s*\(.*placeholders::_1\s*,\s*.*placeholders::_2\s*\)\s*\)"
    assert re.search(pattern, content)

def test_anti_leakage_ros1_artifacts():
    """Ensure no ROS 1 namespaces or legacy patterns survived the migration."""
    full_code = get_content(HPP_FILE) + get_content(CPP_FILE)
    forbidden = ["ros::NodeHandle", "ros::Subscriber", "ros::Publisher", "ros::ok()"]
    for item in forbidden:
        assert item not in full_code, f"Legacy ROS 1 artifact found: {item}"

def test_namespace_migration():
    """Verify the move from flat ROS 1 messages to ROS 2 nested srv/msg namespaces."""
    content = get_content(HPP_FILE)
    assert "turtlesim_msgs::srv::Spawn" in content
    assert "std_srvs::srv::Empty" in content