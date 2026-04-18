import re
import pytest
import os
from pathlib import Path
# 定义文件路径（根据你的目录结构调整）
SERVICE_FILE = Path(__file__).resolve().parents[1]/"set_joint_states_service.py"
CLIENT_FILE = Path(__file__).resolve().parents[1]/"set_joint_states_client.py"
CMAKE_FILE = Path(__file__).resolve().parents[1]/"CMakeLists.txt"
PKG_FILE = Path(__file__).resolve().parents[1]/"package.xml"

def get_content(file_path):
    if not os.path.exists(file_path):
        return ""
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

# --- 1. Service Server Logic Test ---
def test_service_server_migration():
    """
    Checks if the Service Server uses ROS 2 node patterns and correct callback signatures.
    """
    content = get_content(SERVICE_FILE)
    
    # Check for ROS 2 callback signature (request, response)
    assert re.search(r"def\s+\w+\(self,\s*request,\s*response\):", content) or \
           re.search(r"def\s+\w+\(request,\s*response\):", content), \
           "Logic Failure: Service callback must accept (request, response) arguments."
    
    # Check for explicit response return
    assert "return response" in content, "Logic Failure: ROS 2 service callback must return the response object."
    
    # Check for rclpy initialization
    assert "rclpy.init" in content and "create_service" in content, \
        "Architecture Failure: Must use rclpy and Node.create_service."

# --- 2. Async Client Logic Test ---
def test_service_client_async_integrity():
    """
    Checks if the Client handles the service call asynchronously to avoid deadlocks.
    """
    content = get_content(CLIENT_FILE)
    
    # Must use call_async
    assert "call_async" in content, "Architecture Failure: Service calls in ROS 2 should be asynchronous."
    
    # Must wait for future/result
    assert "spin_until_future_complete" in content or ".result()" in content, \
        "Logic Failure: The client must wait for the future to complete or handle the result."
    
    # Ensure no rospy remains
    assert "rospy" not in content.lower(), "Legacy Failure: 'rospy' detected in client script."

# --- 3. CMakeLists.txt Build System Test ---
def test_cmakelists_rosidl_integrity():
    """
    Checks if the build script correctly implements the ROS 2 interface generation pipeline.
    """
    content = get_content(CMAKE_FILE)
    
    # Check for rosidl generation
    assert "rosidl_generate_interfaces" in content, \
        "Build Failure: Missing 'rosidl_generate_interfaces' to compile SetJointStates.srv."
    
    # Check for ROS 2 standard install path
    assert re.search(r"DESTINATION\s+lib/\${PROJECT_NAME}", content), \
        "Install Failure: Python scripts must be installed to lib/${PROJECT_NAME} for ROS 2."
    
    # Check for ament export
    assert "ament_package()" in content, "Build Failure: Missing 'ament_package()' footer."
    
    # Ensure no ROS 1 macros exist
    legacy_macros = ["catkin_package", "add_service_files", "generate_messages"]
    for macro in legacy_macros:
        assert macro not in content, f"Legacy Failure: Found ROS 1 macro '{macro}' in CMakeLists.txt."

# --- 4. package.xml Dependency Test ---
def test_package_xml_modern_format():
    """
    Checks if package.xml follows format 3 and includes necessary interface groups.
    """
    content = get_content(PKG_FILE)
    
    # Check format and build type
    assert 'format="3"' in content, "Format Failure: package.xml must use format='3'."
    assert "<build_type>ament_cmake</build_type>" in content, "Build Failure: build_type must be ament_cmake."
    
    # Check for the critical interface member group
    assert "rosidl_interface_packages" in content, \
        "Dependency Failure: Missing 'rosidl_interface_packages' member group for custom services."
    
    # Check for ROS 2 generators
    assert "rosidl_default_generators" in content, "Dependency Failure: Missing 'rosidl_default_generators'."
    assert "rclpy" in content, "Dependency Failure: Missing 'rclpy' dependency."
    
    # Ensure no rospy remains
    assert "rospy" not in content, "Legacy Failure: 'rospy' detected in package.xml."

# --- 5. Global Cleanup Check ---
def test_no_rospy_anywhere():
    """
    Final check to ensure no ROS 1 residuals exist in the entire package logic.
    """
    all_content = (get_content(SERVICE_FILE) + get_content(CLIENT_FILE) + 
                   get_content(CMAKE_FILE) + get_content(PKG_FILE)).lower()
    
    assert "rospy" not in all_content, "Migration Failure: ROS 1 'rospy' remnants found in the package."
    assert "catkin" not in all_content, "Migration Failure: ROS 1 'catkin' remnants found in the package."
