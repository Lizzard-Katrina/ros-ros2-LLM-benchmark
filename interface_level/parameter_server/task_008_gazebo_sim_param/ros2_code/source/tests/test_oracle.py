import re
from pathlib import Path
import pytest

CPP_FILE = Path(__file__).resolve().parents[1] / "gazebo_ros_camera_utils.cpp"

def get_clean_code():
    if not CPP_FILE.exists():
        return ""
    code = CPP_FILE.read_text(encoding="utf-8")
    code = re.sub(r'//.*', '', code)
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
    return code

def test_node_factory_migration():
    code = get_clean_code()
    pattern = r'gazebo_ros::Node::(?:Get|get)\s*\(_sdf\)'
    assert re.search(pattern, code), "The active code does not correctly initialize the ROS 2 Gazebo Node."

def test_absence_of_ros1_nodehandle():
    code = get_clean_code()
    assert "ros::NodeHandle" not in code, "Detected legacy ros::NodeHandle in the migrated code logic."

def test_parameter_declaration_style():
    code = get_clean_code()
    snake_params = ["update_rate", "camera_name", "frame_name", "distortion_k1"]
    for param in snake_params:
        assert param in code, f"Missing migrated snake_case parameter: {param}"
    assert "declare_parameter" in code, "Failed to use declare_parameter for ROS 2 configuration."

def test_multicamera_suffix_preservation():
    code = get_clean_code()
    assert re.search(r'camera_name_.*?\+=.*?_camera_name_suffix', code) or \
           re.search(r'camera_name_.*?_camera_name_suffix', code), \
        "The model likely lost the '_camera_name_suffix' logic which is critical for Multicamera plugins."

def test_standard_library_pointer_migration():
    code = get_clean_code()
    assert "std::shared_ptr" in code or "std::unique_ptr" in code
    assert "std::mutex" in code or "std::thread" in code
    assert "boost::shared_ptr" not in code, "Found legacy boost::shared_ptr in implementation."
    assert "boost::mutex" not in code, "Found legacy boost::mutex in implementation."

def test_logging_macros_migration():
    code = get_clean_code()
    assert "RCLCPP_DEBUG" in code or "RCLCPP_INFO" in code
    assert "ROS_DEBUG_NAMED" not in code, "Found legacy ROS_DEBUG_NAMED macro."

def test_time_source_logic():
    code = get_clean_code()
    assert "now()" in code
    assert "gazebo_ros_node_->now()" in code or "get_clock()->now()" in code, \
        "Time source should be derived from the ROS 2 Node's clock."

def test_std_function_migration():
    code = get_clean_code()
    assert "std::function" in code or "std::bind" in code
    assert "boost::function" not in code, "Found legacy boost::function."
    assert "boost::bind" not in code, "Found legacy boost::bind."