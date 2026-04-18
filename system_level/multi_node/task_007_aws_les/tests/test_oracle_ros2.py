import re
import pytest
from pathlib import Path

# Path configuration
HEADER_FILE = Path(__file__).resolve().parents[1] / "lex_node.h"
SOURCE_FILE = Path(__file__).resolve().parents[1] / "lex_node.cpp"

def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# --- 1. System Consistency Tests ---

def test_member_variable_consistency():
    """Verify that the service variable name declared in the header is used in the source initialization."""
    h_content = get_content(HEADER_FILE)
    cpp_content = get_content(SOURCE_FILE)
    
    # Extract service variable name from header (expecting suffix _server_)
    decl_match = re.search(r"rclcpp::Service<[^>]+>::SharedPtr\s+(\w+)_server_;", h_content)
    assert decl_match, "Header must declare a ROS 2 Service with suffix '_server_'"
    
    var_name = f"{decl_match.group(1)}_server_"
    
    # Verify the same variable is initialized in .cpp using create_service
    init_pattern = rf"(?:this->)?{var_name}\s*=\s*(?:this->)?create_service"
    assert re.search(init_pattern, cpp_content), f"Implementation in .cpp must initialize the same variable '{var_name}' declared in header"

def test_callback_signature_sync():
    """Verify the callback signature matches between header declaration and source implementation."""
    h_content = get_content(HEADER_FILE)
    cpp_content = get_content(SOURCE_FILE)
    
    # Check for ROS 2 SharedPtr signature in header
    h_cb = re.search(r"void\s+LexServerCallback\s*\(\s*[^,]+Request::SharedPtr[^,]+,\s*[^,]+Response::SharedPtr", h_content)
    assert h_cb, "Header callback signature must use SharedPtr for Request and Response"
    
    # Check for matching implementation in source
    cpp_cb = re.search(r"void\s+LexNode::LexServerCallback\s*\(\s*[^,]+Request::SharedPtr[^,]+,\s*[^,]+Response::SharedPtr", cpp_content)
    assert cpp_cb, "Source implementation signature does not match the Header declaration"

# --- 2. ROS 2 API Patterns ---

def test_ros2_parameter_declaration():
    """Verify that parameters are declared within the constructor body."""
    cpp_content = get_content(SOURCE_FILE)
    
    # Locate constructor body
    constructor_body = re.search(r"LexNode::LexNode\s*\([^)]*\)[^{]*\{([\s\S]*?)\}", cpp_content)
    assert constructor_body, "Could not find LexNode constructor implementation"
    
    assert "this->declare_parameter" in constructor_body.group(1), "ROS 2 parameters should be declared in the constructor"

def test_service_binding_pattern():
    """Verify the use of std::bind with two placeholders for service registration."""
    cpp_content = get_content(SOURCE_FILE)
    
    # Match std::bind pattern for LexServerCallback
    bind_pattern = r"std::bind\s*\(\s*&LexNode::LexServerCallback\s*,\s*this\s*,\s*std::placeholders::_1\s*,\s*std::placeholders::_2\s*\)"
    assert re.search(bind_pattern, cpp_content), "Service initialization must use std::bind with two placeholders (_1, _2)"

def test_inheritance_pattern():
    """Verify the class inherits from rclcpp::Node."""
    h_content = get_content(HEADER_FILE)
    
    assert re.search(r"class\s+LexNode\s*:\s*public\s+rclcpp::Node", h_content), "LexNode should inherit from rclcpp::Node in ROS 2"

# --- 3. Type System & Cleanup ---

def test_message_namespace_migration():
    """Verify migration from ROS 1 message types to ROS 2 srv namespace."""
    h_content = get_content(HEADER_FILE)
    cpp_content = get_content(SOURCE_FILE)
    
    pattern = r"lex_common_msgs::srv::AudioTextConversation"
    assert re.search(pattern, h_content), "Header should use 'lex_common_msgs::srv' namespace"
    assert re.search(pattern, cpp_content), "Source should use 'lex_common_msgs::srv' namespace"

def test_ros1_leakage_cleanup():
    """Ensure ROS 1 specific classes and methods are removed."""
    combined_content = get_content(HEADER_FILE) + get_content(SOURCE_FILE)
    
    ros1_artifacts = [
        r"ros::NodeHandle",
        r"ros::ServiceServer",
        r"advertiseService",
        r"ros::init",
        r"operator\s+ros::NodeHandle"
    ]
    
    for artifact in ros1_artifacts:
        assert not re.search(artifact, combined_content, re.IGNORECASE), f"Found ROS 1 artifact leakage: {artifact}"

def test_defensive_programming_retention():
    """Verify that the defensive null check for post_content is preserved in Init."""
    cpp_content = get_content(SOURCE_FILE)
    
    init_body = re.search(r"ErrorCode\s+LexNode::Init\s*\([\s\S]*?\{([\s\S]*?)\}", cpp_content)
    assert init_body, "Could not find Init function body"
    
    assert re.search(r"if\s*\(\s*!post_content\s*\)", init_body.group(1)), "Defensive null-check for 'post_content' was lost during migration"
