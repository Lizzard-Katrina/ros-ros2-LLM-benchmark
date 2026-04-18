import re
import pytest
from pathlib import Path

# Path resolution
ROOT_DIR = Path(__file__).resolve().parents[1]
CMAKE_PATH = ROOT_DIR / "CMakeLists.txt"
CPP_PATH = ROOT_DIR /"gazebo_imu_plugin.cpp"
BUFFER_PATH = ROOT_DIR /"msgbuffer.h"

def get_content(path):
    if not path.exists():
        return ""
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


# --- 1. CMakeLists.txt: ROS 2 & Ament Migration ---

def test_cmake_ros2_ament_integrity():
    """Verify CMake correctly integrates with ROS 2 and Gazebo ROS."""
    content = get_content(CMAKE_PATH)
    assert "find_package(ament_cmake REQUIRED)" in content
    assert "find_package(rclcpp REQUIRED)" in content
    assert "find_package(gazebo_ros REQUIRED)" in content
    # Ensure sensor_msgs is linked via ament macro
    assert re.search(r"ament_target_dependencies\s*\(\s*gazebo_imu_plugin.*sensor_msgs", content, re.DOTALL)
    assert "ament_package()" in content

# --- 2. gazebo_imu_plugin.cpp: Physics & API Migration ---

def test_imu_physics_frequency_scaling_semantics():
    """Verify white noise scaling (1/sqrt(dt)) remains in the migrated code."""
    content = get_content(CPP_PATH)
    # The IMU measurement logic must scale noise by the square root of frequency/dt
    # Matches: 1/sqrt(dt) or density / sqrt(dt)
    assert re.search(r"1\s*/\s*sqrt\s*\(\s*dt\s*\)", content), \
        "Missing 1/sqrt(dt) scaling for white noise density."

def test_gravity_coordinate_transform():
    """Verify gravity is rotated into the link frame before subtraction."""
    content = get_content(CPP_PATH)
    # Correct IMU physics: Accel_measured = Accel_linear - Rotated_Gravity
    assert "RotateVectorReverse" in content
    assert re.search(r"-\s*C_W_I", content), "Gravity subtraction must use the rotated orientation."

def test_ros2_api_usage():
    """Verify migration from ros::NodeHandle to rclcpp::Node."""
    content = get_content(CPP_PATH)
    assert "rclcpp::init" in content
    assert "rclcpp::Node::make_shared" in content
    assert "create_publisher<sensor_msgs::msg::Imu>" in content
    assert "imu_pub_->publish" in content

def test_no_legacy_remnants():
    """Ensure ROS 1 and old Gazebo Protobuf symbols are purged."""
    content = get_content(CPP_PATH)
    legacy = ["ros::NodeHandle", "ros::Publisher", "#include <ros/ros.h>", "set_allocated_"]
    for sym in legacy:
        assert sym not in content

# --- 3. msgbuffer.h: Low-Level Memory & MAVLink Handling ---

def test_msgbuffer_memory_logic():
    """Verify the MAVLink buffer implements correct size and safety logic."""
    content = get_content(BUFFER_PATH)
    
    # 1. Check for MAX_SIZE definition including padding
    assert "MAVLINK_MAX_PACKET_LEN" in content
    assert "MAX_SIZE" in content
    
    # 2. Check for MAVLink specific serialization helper
    assert "mavlink_msg_to_send_buffer" in content
    
    # 3. Check for raw memory copy implementation
    assert re.search(r"memcpy\s*\(\s*data\s*,\s*bytes\s*,\s*nbytes\s*\)", content)

def test_msgbuffer_safety_assertions():
    """Check for bounds checking to prevent overflows in communication buffers."""
    content = get_content(BUFFER_PATH)
    # Ensure assertions protect against oversized packets
    assert "assert" in content
    assert "len < MAX_SIZE" in content or "nbytes < MAX_SIZE" in content

# --- 4. Cross-File Consistency ---

def test_header_inclusion_semantics():
    """Verify correct header inclusion across files."""
    cpp_content = get_content(CPP_PATH)
    # Should include its own header, but not the legacy msgbuffer directly unless needed
    assert '#include "gazebo_imu_plugin.h"' in cpp_content
    
    buffer_content = get_content(BUFFER_PATH)
    assert "namespace gazebo" in buffer_content
