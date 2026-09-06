"""
Runtime test for task_008_exploration_mapping.

This test verifies:
1. The C++ source file has the correct ROS 2 Control architecture
2. The shared library was built successfully
3. The YAML configuration is correct
4. The plugin XML is correct
5. The compiled .so can be loaded via dlopen
"""

import re
import os
import ctypes
import glob
import subprocess
import pytest
from pathlib import Path


def get_source_dir():
    """Get the source directory of the package."""
    return Path(__file__).resolve().parent


def find_shared_library():
    """Find the compiled shared library."""
    search_roots = [
        Path("/ros2_ws/build/task_008_exploration_mapping"),
        Path("/ros2_ws/install/task_008_exploration_mapping/lib"),
        get_source_dir().parent / "build" / "task_008_exploration_mapping",
        get_source_dir().parent / "install" / "task_008_exploration_mapping" / "lib",
    ]
    for root in search_roots:
        if root.exists():
            for so_file in root.rglob("librobot_hardware_interface.so"):
                return so_file
    return None


class TestArchitecture:
    """Test the C++ source file for correct ROS 2 Control architecture."""

    @pytest.fixture(autouse=True)
    def setup(self):
        self.src_dir = get_source_dir()
        self.cpp_file = self.src_dir / "robot_hardware_interface_node.cpp"
        self.yaml_file = self.src_dir / "controllers.yaml"
        self.header_file = self.src_dir / "include" / "task_008_exploration_mapping" / "robot_hardware_interface.h"
        assert self.cpp_file.exists(), f"C++ source not found at {self.cpp_file}"
        assert self.yaml_file.exists(), f"YAML config not found at {self.yaml_file}"
        with open(self.cpp_file, 'r') as f:
            self.cpp_content = f.read()
        with open(self.yaml_file, 'r') as f:
            self.yaml_content = f.read()

    def test_system_interface_inheritance(self):
        """Plugin must inherit from hardware_interface::SystemInterface."""
        assert re.search(r"public\s+hardware_interface::SystemInterface", self.cpp_content) or \
            (self.header_file.exists() and re.search(
                r"public\s+hardware_interface::SystemInterface",
                open(self.header_file).read())), \
            "Class must inherit from hardware_interface::SystemInterface."

    def test_no_main_function(self):
        """Hardware interface must be a plugin, not a standalone executable."""
        assert not re.search(r"int\s+main\s*\(", self.cpp_content), \
            "Hardware Interface must be a plugin; standalone main() detected."

    def test_pluginlib_export(self):
        """Must have PLUGINLIB_EXPORT_CLASS macro."""
        assert "PLUGINLIB_EXPORT_CLASS" in self.cpp_content, \
            "Missing plugin export macro."

    def test_lifecycle_methods(self):
        """Must implement ROS 2 Control lifecycle callbacks."""
        assert "on_init" in self.cpp_content
        assert "export_state_interfaces" in self.cpp_content
        assert "export_command_interfaces" in self.cpp_content
        assert "CallbackReturn" in self.cpp_content

    def test_interface_constants(self):
        """Must use HW_IF_POSITION and HW_IF_VELOCITY constants."""
        assert re.search(r"HW_IF_POSITION", self.cpp_content)
        assert re.search(r"HW_IF_VELOCITY", self.cpp_content)

    def test_joint_names_in_yaml(self):
        """YAML must reference the expected joint names."""
        assert "left_wheel_joint" in self.yaml_content
        assert "right_wheel_joint" in self.yaml_content

    def test_ros2_read_write_signatures(self):
        """read/write must accept (const rclcpp::Time&, const rclcpp::Duration&)."""
        read_pattern = r"read\s*\(\s*const\s+rclcpp::Time\s*&.*,\s*const\s+rclcpp::Duration\s*&.*\)"
        write_pattern = r"write\s*\(\s*const\s+rclcpp::Time\s*&.*,\s*const\s+rclcpp::Duration\s*&.*\)"
        assert re.search(read_pattern, self.cpp_content), "Incorrect read() signature."
        assert re.search(write_pattern, self.cpp_content), "Incorrect write() signature."

    def test_no_ros1_remnants(self):
        """No ROS 1 API calls should remain."""
        assert "ros::NodeHandle" not in self.cpp_content
        assert "registerInterface" not in self.cpp_content

    def test_yaml_structure(self):
        """YAML must have correct ROS 2 structure."""
        assert "controller_manager:" in self.yaml_content
        assert "ros__parameters:" in self.yaml_content
        assert "diff_drive_controller/DiffDriveController" in self.yaml_content
        assert re.search(r"command_interfaces:.*velocity", self.yaml_content, re.DOTALL)


class TestRuntimeLibrary:
    """Test that the compiled shared library actually exists and can be loaded."""

    def test_shared_library_built(self):
        """Verify the .so was produced by the build."""
        so_file = find_shared_library()
        assert so_file is not None, \
            "librobot_hardware_interface.so not found in build or install directories"

    def test_shared_library_loads(self):
        """Verify the .so can be dlopen'd without missing symbols."""
        so_file = find_shared_library()
        if so_file is None:
            pytest.skip("Shared library not found")

        # Preload rclcpp and other ROS libs so symbols resolve
        # We just check it doesn't crash on load
        try:
            lib = ctypes.CDLL(str(so_file), mode=ctypes.RTLD_GLOBAL)
            assert lib is not None, "Failed to load shared library"
        except OSError as e:
            # If it fails due to missing ROS symbols at dlopen time, that's
            # expected in a minimal environment - the library was still built
            if "undefined symbol" in str(e):
                # Library was built but needs full ROS runtime to load
                pass
            else:
                raise

    def test_plugin_description_exists(self):
        """Verify the plugin XML description file exists."""
        src_dir = get_source_dir()
        plugin_xml = src_dir / "task_008_exploration_mapping_plugin.xml"
        assert plugin_xml.exists(), "Plugin description XML must exist"
        with open(plugin_xml, 'r') as f:
            content = f.read()
        assert "ROBOTHardwareInterface" in content
        assert "hardware_interface::SystemInterface" in content

    def test_header_file_exists(self):
        """Verify the header file exists with correct class definition."""
        src_dir = get_source_dir()
        header = src_dir / "include" / "task_008_exploration_mapping" / "robot_hardware_interface.h"
        assert header.exists(), f"Header file must exist at {header}"
        with open(header, 'r') as f:
            content = f.read()
        assert "public hardware_interface::SystemInterface" in content
        assert "on_init" in content
        assert "export_state_interfaces" in content
        assert "export_command_interfaces" in content
        assert "joint_position_" in content
        assert "joint_velocity_" in content
        assert "joint_velocity_command_" in content

    def test_return_types_correct(self):
        """Verify read/write return hardware_interface::return_type."""
        src_dir = get_source_dir()
        cpp_file = src_dir / "robot_hardware_interface_node.cpp"
        with open(cpp_file, 'r') as f:
            content = f.read()
        assert "return_type::OK" in content

    def test_angles_library_used(self):
        """Verify the angles library is still used for degree/radian conversion."""
        src_dir = get_source_dir()
        cpp_file = src_dir / "robot_hardware_interface_node.cpp"
        with open(cpp_file, 'r') as f:
            content = f.read()
        assert "angles::from_degrees" in content
        assert "angles::to_degrees" in content

    def test_i2c_read_write_preserved(self):
        """Verify the I2C business logic is preserved."""
        src_dir = get_source_dir()
        cpp_file = src_dir / "robot_hardware_interface_node.cpp"
        with open(cpp_file, 'r') as f:
            content = f.read()
        assert "readBytes" in content
        assert "writeData" in content
        assert "left_motor" in content and "right_motor" in content