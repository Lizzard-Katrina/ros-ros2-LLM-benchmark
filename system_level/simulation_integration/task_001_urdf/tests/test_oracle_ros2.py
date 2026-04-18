import re
from pathlib import Path
import pytest

# Path definitions - assuming tests are in a 'test' directory and source is in 'urdf'/'launch'
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "display.launch"
URDF_FILE = Path(__file__).resolve().parents[1] / "macroed.urdf.xacro"

class TestSimulationIntegration:
    @classmethod
    def get_file_content(cls, file_path):
        if not file_path.exists():
            pytest.fail(f"File not found: {file_path}")
        with open(file_path, "r") as f:
            return f.read()

    def test_urdf_base_link_consistency(self):
        """Verifies base_link reconstruction and use of Xacro properties."""
        content = self.get_file_content(URDF_FILE)
        # Matches link name 'base_link' containing a cylinder with parameterized radius and length
        pattern = r'<link\s+name=["\']base_link["\']>.*?<cylinder\s+radius=["\']\$\{width\}["\']\s+length=["\']\$\{bodylen\}["\']'
        assert re.search(pattern, content, re.DOTALL), \
            "base_link definition is missing or fails to use '${width}' and '${bodylen}' properties."

    def test_urdf_inertial_integration(self):
        """Verifies that physics/inertial macros are integrated for simulation."""
        content = self.get_file_content(URDF_FILE)
        # Matches the call to default_inertial macro with mass 10 inside the base_link scope
        pattern = r'<link\s+name=["\']base_link["\']>(?:(?!</link>).)*?<xacro:default_inertial\s+mass=["\']10["\']\s*/>'
        assert re.search(pattern, content, re.DOTALL), \
            "base_link is missing the required inertial macro call with mass='10'."

    def test_launch_xacro_command_api(self):
        """Verifies the use of ROS 2 Command API for dynamic Xacro processing."""
        content = self.get_file_content(LAUNCH_FILE)
        # Ensures 'xacro' executable is invoked via the Command substitution
        pattern = r"Command\(\s*\[\s*['\"]xacro['\"]\s*,"
        assert re.search(pattern, content), \
            "Launch file must use 'launch.substitutions.Command' to process the Xacro file."

    def test_launch_resource_indexing(self):
        """Verifies ROS 2 compliant package resource indexing."""
        content = self.get_file_content(LAUNCH_FILE)
        # Matches ROS 2 specific package finding methods
        pattern = r"(?:get_package_share_directory|FindPackageShare)\(\s*['\"]urdf_tutorial['\"]"
        assert re.search(pattern, content), \
            "Launch file should use 'get_package_share_directory' or 'FindPackageShare' for package indexing."

    def test_launch_parameter_wrapping(self):
        """Verifies that robot_description is correctly wrapped for the parameter system."""
        content = self.get_file_content(LAUNCH_FILE)
        # Checks for ParameterValue wrapping to handle XML strings safely
        pattern = r"['\"]robot_description['\"]\s*:\s*(?:[^,}]*?ParameterValue)"
        assert re.search(pattern, content), \
            "The 'robot_description' must be wrapped in 'ParameterValue' for proper XML string handling."

    def test_anti_leakage_ros1_substitution(self):
        """Ensures no legacy ROS 1 substitution syntax remains."""
        content = self.get_file_content(LAUNCH_FILE)
        # Checks for the absence of '$(find ...)'
        ros1_syntax = r"\$\(find\s+.*?\)"
        assert not re.search(ros1_syntax, content), \
            "Found legacy ROS 1 '$(find ...)' syntax. Use ROS 2 substitutions instead."

    def test_launch_integration_completeness(self):
        """Verifies the inclusion of the downstream display launch action."""
        content = self.get_file_content(LAUNCH_FILE)
        # Ensures the 'display.launch.py' from urdf_launch is included
        pattern = r"IncludeLaunchDescription\s*\(.*?display\.launch\.py"
        assert re.search(pattern, content, re.DOTALL), \
            "The launch file is missing the inclusion of the 'display.launch.py' action."
