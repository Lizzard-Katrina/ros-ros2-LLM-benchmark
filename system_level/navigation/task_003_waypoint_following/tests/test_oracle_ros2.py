import re
import pytest
from pathlib import Path

# Path resolution: script is in src/test/, so parents[1] is the task root
BASE_DIR = Path(__file__).resolve().parents[1] 

CPP_FILE =BASE_DIR/"add_two_ints_server.cpp"
CMAKE_FILE =BASE_DIR/"CMakeLists.txt"
XML_FILE =BASE_DIR/"package.xml"

def get_content(file_path):
    return file_path.read_text() if file_path.exists() else ""

# --- Consistency: Package.xml ↔ CMakeLists.txt ---

def test_dependency_sync_xml_and_cmake():
    """Verify that dependencies are declared in both package.xml and CMake."""
    xml = get_content(XML_FILE)
    cmake = get_content(CMAKE_FILE)
    
    sync_deps = ['rclcpp', 'rclcpp_action', 'action_msgs']
    for dep in sync_deps:
        # Check XML (format 3 usually uses <depend>)
        xml_check = f"<depend>{dep}</depend>" in xml or f"<{dep}" in xml
        # Check CMake find_package
        cmake_check = f"find_package({dep}" in cmake.lower()
        
        assert xml_check and cmake_check, f"Dependency '{dep}' is not synchronized between package.xml and CMake."

# --- Consistency: CMakeLists.txt ↔ C++ ---

def test_interface_sync_cmake_and_cpp():
    """Verify that CMake generates the interfaces that C++ includes."""
    cmake = get_content(CMAKE_FILE)
    cpp = get_content(CPP_FILE)
    
    if "two_ints.hpp" in cpp.lower():
        assert "rosidl_generate_interfaces" in cmake, \
            "C++ uses actions but CMake does not call rosidl_generate_interfaces."
    
    if "rclcpp_action" in cpp:
        assert "ament_target_dependencies" in cmake or "target_link_libraries" in cmake, \
            "C++ uses rclcpp_action but CMake does not link it."

# --- Logic: C++ Implementation ---

def test_cpp_action_server_logic():
    """Verify ROS2 Action Server logic and core arithmetic."""
    cpp = get_content(CPP_FILE)
    
    # Check for ROS2 specifics
    assert "ServerGoalHandle" in cpp, "C++ code failed to use ServerGoalHandle."
    assert "rclcpp_action::create_server" in cpp or "rclcpp_action::Server" in cpp, \
        "Action Server initialization is missing."
    
    # Check for functional parity: result = a + b
    assert re.search(r"\.a\s*\+\s*.*\.b", cpp), "Core logic 'a + b' is missing from C++ server."

# --- Build tool check ---

def test_build_system_migration():
    """Ensure the entire build toolchain moved from catkin to ament."""
    xml = get_content(XML_FILE)
    cmake = get_content(CMAKE_FILE)
    
    assert "ament_cmake" in xml, "package.xml still references legacy build tools."
    assert "ament_package()" in cmake, "CMakeLists.txt is missing ament_package()."
    assert "catkin" not in cmake.lower(), "Legacy 'catkin' keyword found in CMakeLists.txt."
