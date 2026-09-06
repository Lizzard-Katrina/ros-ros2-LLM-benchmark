import re
from pathlib import Path
import pytest

# Path definitions
CPP_FILE = Path(__file__).resolve().parents[1] / "simple_world_plugin.cpp"
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "hello.launch"

class TestGazeboPluginTask:
    @classmethod
    def get_content(cls, path):
        if not path.exists():
            pytest.fail(f"File not found: {path}")
        with open(path, "r") as f:
            return f.read()

    def test_cpp_load_function_signature(self):
        """Verifies that the Load function is implemented with correct Gazebo pointers."""
        content = self.get_content(CPP_FILE)
        pattern = r"void\s+Load\s*\(\s*physics::WorldPtr\s+\w+,\s*sdf::ElementPtr\s+\w+\s*\)"
        assert re.search(pattern, content), \
            "The Load() function signature is missing or does not include required WorldPtr/ElementPtr."

    def test_cpp_plugin_registration(self):
        """Ensures the plugin is registered using the mandatory Gazebo macro."""
        content = self.get_content(CPP_FILE)
        assert "GZ_REGISTER_WORLD_PLUGIN" in content, \
            "The plugin registration macro 'GZ_REGISTER_WORLD_PLUGIN' is missing."

    def test_cpp_namespace_consistency(self):
        """Checks if the plugin remains within the gazebo namespace as required."""
        content = self.get_content(CPP_FILE)
        assert "namespace gazebo" in content.lower(), \
            "The plugin implementation should be wrapped inside the 'gazebo' namespace."

    def test_launch_empty_world_inclusion(self):
        """Verifies the launch file correctly includes the base gazebo_ros empty_world."""
        content = self.get_content(LAUNCH_FILE)
        pattern = r'<include\s+file=["\']\$\(find\s+gazebo_ros\)/launch/empty_world\.launch["\']'
        assert re.search(pattern, content), \
            "The launch file must include 'empty_world.launch' from the gazebo_ros package."

    def test_launch_world_parameter_passing(self):
        """Validates that the custom hello.world file path is passed as an argument."""
        content = self.get_content(LAUNCH_FILE)
        pattern = r'<arg\s+name=["\']world_name["\']\s+value=["\']\$\(find\s+gazebo_tutorials\)/worlds/hello\.world["\']'
        assert re.search(pattern, content), \
            "The 'world_name' argument must correctly point to '$(find gazebo_tutorials)/worlds/hello.world'."

    def test_launch_xml_structure(self):
        """Basic check to ensure the launch file maintains valid ROS 1 XML tags."""
        content = self.get_content(LAUNCH_FILE)
        assert content.strip().startswith("<launch>") and content.strip().endswith("</launch>"), \
            "The launch file must be a valid ROS 1 XML structure starting and ending with <launch> tags."

    def test_anti_leakage_ros2_substitution(self):
        """Ensures no ROS 2 Python-style launch logic is leaked into the ROS 1 XML file."""
        content = self.get_content(LAUNCH_FILE)
        bad_keywords = ["LaunchDescription", "get_package_share_directory", "Node("]
        for word in bad_keywords:
            assert word not in content, f"Found unexpected ROS 2 syntax keyword: {word}"