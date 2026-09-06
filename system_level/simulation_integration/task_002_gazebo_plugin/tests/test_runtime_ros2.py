"""
Runtime test for task_002_gazebo_plugin.

Validates the actual source files from the package have correct structure,
API signatures, and ROS2 migration patterns.
"""

import os
import re
from pathlib import Path

import pytest


def find_package_root():
    """Find the package root by looking for package.xml."""
    test_dir = Path(__file__).resolve().parent
    if (test_dir / "package.xml").exists():
        return test_dir
    current = test_dir
    for _ in range(5):
        if (current / "package.xml").exists():
            return current
        current = current.parent
    pytest.fail("Could not find package root with package.xml")


class TestGazeboPluginSource:
    """Tests that validate the actual translated source files."""

    @classmethod
    def setup_class(cls):
        cls.pkg_root = find_package_root()

    def _read_file(self, filename):
        """Read a file from the package, checking multiple locations."""
        candidates = [
            self.pkg_root / filename,
            self.pkg_root / "src" / filename,
            self.pkg_root / "launch" / filename,
        ]
        for p in candidates:
            if p.exists():
                return p.read_text()
        pytest.fail(f"Could not find {filename} in package")

    # ---- C++ source tests ----

    def test_cpp_source_load_function_signature(self):
        """Verify the actual source file has the correct Load function signature."""
        content = self._read_file("simple_world_plugin.cpp")
        pattern = r"void\s+Load\s*\(\s*physics::WorldPtr\s+\w+,\s*sdf::ElementPtr\s+\w+\s*\)"
        assert re.search(pattern, content), \
            "The Load() function signature must include physics::WorldPtr and sdf::ElementPtr"

    def test_cpp_source_plugin_registration(self):
        """Verify the actual source file has the GZ_REGISTER_WORLD_PLUGIN macro."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "GZ_REGISTER_WORLD_PLUGIN" in content, \
            "Missing GZ_REGISTER_WORLD_PLUGIN macro"
        assert "WorldPluginTutorial" in content, \
            "Missing WorldPluginTutorial class name in registration"

    def test_cpp_source_namespace(self):
        """Verify the plugin is in the gazebo namespace."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "namespace gazebo" in content, \
            "Plugin must be in the gazebo namespace"

    def test_cpp_source_world_plugin_inheritance(self):
        """Verify the class inherits from WorldPlugin."""
        content = self._read_file("simple_world_plugin.cpp")
        assert re.search(r"class\s+WorldPluginTutorial\s*:\s*public\s+WorldPlugin", content), \
            "Class must inherit from WorldPlugin"

    def test_cpp_source_includes_gazebo(self):
        """Verify required gazebo headers are included."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "#include" in content, "Must have include directives"
        assert "gazebo" in content.lower(), "Must include gazebo headers"

    def test_cpp_source_uses_rclcpp(self):
        """Verify the ROS2 migration uses rclcpp instead of roscpp."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "rclcpp" in content, \
            "ROS2 plugin must use rclcpp"

    def test_cpp_source_uses_rclcpp_logging(self):
        """Verify the ROS2 migration uses RCLCPP_INFO instead of ROS_INFO."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "RCLCPP_INFO" in content, \
            "ROS2 plugin must use RCLCPP_INFO for logging"

    def test_cpp_source_no_ros1_api(self):
        """Verify no ROS1 API remnants."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "ros::isInitialized" not in content, \
            "Should not use ros::isInitialized (ROS1 API)"
        assert "ROS_INFO(" not in content, \
            "Should not use ROS_INFO (ROS1 API) - use RCLCPP_INFO"
        assert "ROS_FATAL_STREAM" not in content, \
            "Should not use ROS_FATAL_STREAM (ROS1 API)"

    def test_cpp_source_hello_world_message(self):
        """Verify the Hello World message is present."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "Hello World" in content, \
            "Must contain 'Hello World' message"

    def test_cpp_source_stores_world_ptr(self):
        """Verify the plugin stores the world pointer."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "physics::WorldPtr" in content, \
            "Must store physics::WorldPtr"

    def test_cpp_source_stores_sdf_ptr(self):
        """Verify the plugin stores the SDF element pointer."""
        content = self._read_file("simple_world_plugin.cpp")
        assert "sdf::ElementPtr" in content, \
            "Must store sdf::ElementPtr"

    # ---- Launch file tests ----

    def test_launch_file_exists_and_valid_xml(self):
        """Verify the launch file exists and has valid ROS1 XML structure."""
        content = self._read_file("hello.launch")
        assert content.strip().startswith("<launch>"), \
            "Launch file must start with <launch>"
        assert content.strip().endswith("</launch>"), \
            "Launch file must end with </launch>"

    def test_launch_empty_world_inclusion(self):
        """Verify the launch file includes empty_world.launch from gazebo_ros."""
        content = self._read_file("hello.launch")
        pattern = r'<include\s+file=["\']\$\(find\s+gazebo_ros\)/launch/empty_world\.launch["\']'
        assert re.search(pattern, content), \
            "Must include $(find gazebo_ros)/launch/empty_world.launch"

    def test_launch_world_name_argument(self):
        """Verify the world_name argument points to the correct world file."""
        content = self._read_file("hello.launch")
        pattern = r'<arg\s+name=["\']world_name["\']\s+value=["\']\$\(find\s+gazebo_tutorials\)/worlds/hello\.world["\']'
        assert re.search(pattern, content), \
            "Must pass world_name argument pointing to $(find gazebo_tutorials)/worlds/hello.world"

    # ---- World file tests ----

    def test_world_file_exists(self):
        """Verify the hello.world SDF file exists."""
        world_path = self.pkg_root / "worlds" / "hello.world"
        assert world_path.exists(), "worlds/hello.world must exist in the package"

    def test_world_file_references_plugin(self):
        """Verify the world file references the simple_world_plugin."""
        world_path = self.pkg_root / "worlds" / "hello.world"
        if not world_path.exists():
            pytest.skip("World file not found")
        content = world_path.read_text()
        assert "simple_world_plugin" in content, \
            "World file should reference the simple_world_plugin"
        assert "<sdf" in content, "World file should be valid SDF"

    # ---- CMakeLists.txt tests ----

    def test_cmake_references_plugin(self):
        """Verify CMakeLists.txt references the plugin target."""
        content = (self.pkg_root / "CMakeLists.txt").read_text()
        assert "simple_world_plugin" in content, \
            "CMakeLists.txt must reference simple_world_plugin"
        assert "SHARED" in content, \
            "Must build as SHARED library"

    def test_cmake_uses_ament(self):
        """Verify CMakeLists.txt uses ament_cmake."""
        content = (self.pkg_root / "CMakeLists.txt").read_text()
        assert "ament_cmake" in content, \
            "CMakeLists.txt must use ament_cmake"
        assert "ament_package" in content, \
            "CMakeLists.txt must call ament_package()"

    # ---- Package.xml tests ----

    def test_package_xml_format3(self):
        """Verify package.xml uses format 3."""
        content = (self.pkg_root / "package.xml").read_text()
        assert 'format="3"' in content, \
            "package.xml must use format 3"

    def test_package_xml_build_type(self):
        """Verify package.xml declares ament_cmake build type."""
        content = (self.pkg_root / "package.xml").read_text()
        assert "ament_cmake" in content, \
            "package.xml must declare ament_cmake build type"