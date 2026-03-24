import re
from pathlib import Path
import pytest

# Path resolution to the source file
CPP_FILE = Path(__file__).resolve().parents[1] / "gazebo_ros_camera_utils.cpp"

def get_clean_code():
    """
    Strips comments from the source code to ensure regex matches only 
    active implementation, avoiding false positives from TODO descriptions.
    """
    if not CPP_FILE.exists():
        return ""
    code = CPP_FILE.read_text(encoding="utf-8")
    # Remove single-line comments // ...
    code = re.sub(r'//.*', '', code)
    # Remove multi-line comments /* ... */
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
    return code

# --- 1. Node & Infrastructure Migration ---

def test_node_factory_migration():
    """Concept: Uses the ROS 2 gazebo_ros::Node factory (Get/get) instead of NodeHandle."""
    code = get_clean_code()
    # Matches: gazebo_ros::Node::Get(_sdf) or gazebo_ros::Node::get(_sdf)
    pattern = r'gazebo_ros::Node::(?:Get|get)\s*\(_sdf\)'
    assert re.search(pattern, code), "The active code does not correctly initialize the ROS 2 Gazebo Node."

def test_absence_of_ros1_nodehandle():
    """Concept: Ensures no active code uses the legacy ros::NodeHandle."""
    code = get_clean_code()
    assert "ros::NodeHandle" not in code, "Detected legacy ros::NodeHandle in the migrated code logic."

# --- 2. Parameter & Naming Logic ---

def test_parameter_declaration_style():
    """Concept: Uses declare_parameter with snake_case naming conventions."""
    code = get_clean_code()
    # Check for core parameters converted to snake_case
    snake_params = ["update_rate", "camera_name", "frame_name", "distortion_k1"]
    for param in snake_params:
        assert param in code, f"Missing migrated snake_case parameter: {param}"
    assert "declare_parameter" in code, "Failed to use declare_parameter for ROS 2 configuration."

def test_multicamera_suffix_preservation():
    """Concept: Verifies the camera name suffix is not lost during migration."""
    code = get_clean_code()
    # Check if _camera_name_suffix is appended to the final camera_name_
    # This is a common failure point where LLMs overwrite the name entirely.
    assert re.search(r'camera_name_.*?\+=.*?_camera_name_suffix', code) or \
           re.search(r'camera_name_.*?_camera_name_suffix', code), \
        "The model likely lost the '_camera_name_suffix' logic which is critical for Multicamera plugins."

# --- 3. Synchronisation & Threading ---

def test_standard_library_pointer_migration():
    """Concept: Replaces boost::shared_ptr and boost::mutex with C++ standard library."""
    code = get_clean_code()
    assert "std::shared_ptr" in code or "std::unique_ptr" in code
    assert "std::mutex" in code or "std::thread" in code
    assert "boost::shared_ptr" not in code, "Found legacy boost::shared_ptr in implementation."
    assert "boost::mutex" not in code, "Found legacy boost::mutex in implementation."

# --- 4. Middleware & Logging ---

def test_logging_macros_migration():
    """Concept: Replaces ROS 1 ROS_DEBUG/INFO with ROS 2 RCLCPP equivalents."""
    code = get_clean_code()
    assert "RCLCPP_DEBUG" in code or "RCLCPP_INFO" in code
    assert "ROS_DEBUG_NAMED" not in code, "Found legacy ROS_DEBUG_NAMED macro."

def test_time_source_logic():
    """Concept: Ensures the code uses the Node's clock for simulation-time compatibility."""
    code = get_clean_code()
    # ROS 2 Gazebo plugins should use the node's clock for /use_sim_time support
    assert "now()" in code
    assert "gazebo_ros_node_->now()" in code or "get_clock()->now()" in code, \
        "Time source should be derived from the ROS 2 Node's clock."

# --- 5. Callback & Event Logic ---

def test_std_function_migration():
    """Concept: Replaces boost::function/bind with std::function/bind."""
    code = get_clean_code()
    assert "std::function" in code or "std::bind" in code
    assert "boost::function" not in code, "Found legacy boost::function."
    assert "boost::bind" not in code, "Found legacy boost::bind."
