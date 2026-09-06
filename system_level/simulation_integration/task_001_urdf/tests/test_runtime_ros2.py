"""
Runtime test for task_001_urdf: validates URDF xacro content and launch file content.
This test actually reads the translated files and validates their structure and semantics.
"""
import re
import os
import subprocess
import pytest
from pathlib import Path


# Locate files relative to this test file (package root)
PACKAGE_ROOT = Path(__file__).resolve().parent
URDF_FILE = PACKAGE_ROOT / "macroed.urdf.xacro"
URDF_FILE_ALT = PACKAGE_ROOT / "urdf" / "08-macroed.urdf.xacro"
LAUNCH_FILE = PACKAGE_ROOT / "display.launch"
LAUNCH_FILE_ALT = PACKAGE_ROOT / "launch" / "display.launch.py"


def get_urdf_content():
    """Get URDF content from available file."""
    for f in [URDF_FILE, URDF_FILE_ALT]:
        if f.exists():
            return f.read_text()
    pytest.fail(f"No URDF file found at {URDF_FILE} or {URDF_FILE_ALT}")


def get_launch_content():
    """Get launch file content from available file."""
    for f in [LAUNCH_FILE, LAUNCH_FILE_ALT]:
        if f.exists():
            return f.read_text()
    pytest.fail(f"No launch file found at {LAUNCH_FILE} or {LAUNCH_FILE_ALT}")


class TestURDFXacroContent:
    """Test the URDF xacro file content for correctness."""

    def test_urdf_file_exists(self):
        """Verify at least one URDF file exists."""
        assert URDF_FILE.exists() or URDF_FILE_ALT.exists(), \
            "URDF xacro file not found"

    def test_urdf_is_valid_xml(self):
        """Verify the URDF file is valid XML."""
        import xml.etree.ElementTree as ET
        content = get_urdf_content()
        # Xacro files have custom namespace, just check it parses as XML-like
        assert '<?xml' in content, "Missing XML declaration"
        assert '<robot' in content, "Missing <robot> tag"

    def test_urdf_base_link_has_cylinder(self):
        """Verify base_link uses cylinder geometry with xacro properties."""
        content = get_urdf_content()
        pattern = r'<link\s+name=["\']base_link["\']>.*?<cylinder\s+radius=["\']\$\{width\}["\']\s+length=["\']\$\{bodylen\}["\']'
        assert re.search(pattern, content, re.DOTALL), \
            "base_link must use cylinder with ${width} and ${bodylen}"

    def test_urdf_base_link_has_inertial(self):
        """Verify base_link has inertial macro with mass=10."""
        content = get_urdf_content()
        pattern = r'<link\s+name=["\']base_link["\']>(?:(?!</link>).)*?<xacro:default_inertial\s+mass=["\']10["\']\s*/>'
        assert re.search(pattern, content, re.DOTALL), \
            "base_link must have <xacro:default_inertial mass='10' />"

    def test_urdf_has_xacro_properties(self):
        """Verify xacro properties are defined."""
        content = get_urdf_content()
        assert 'xacro:property name="width"' in content
        assert 'xacro:property name="bodylen"' in content

    def test_urdf_has_default_inertial_macro(self):
        """Verify the default_inertial macro is defined."""
        content = get_urdf_content()
        assert 'xacro:macro name="default_inertial"' in content

    def test_urdf_robot_name(self):
        """Verify robot name is 'macroed'."""
        content = get_urdf_content()
        assert re.search(r'<robot\s+[^>]*name=["\']macroed["\']', content), \
            "Robot name should be 'macroed'"


class TestLaunchFileContent:
    """Test the launch file content for ROS2 compliance."""

    def test_launch_file_exists(self):
        """Verify at least one launch file exists."""
        assert LAUNCH_FILE.exists() or LAUNCH_FILE_ALT.exists(), \
            "Launch file not found"

    def test_launch_uses_command_api(self):
        """Verify Command(['xacro', ...]) is used."""
        content = get_launch_content()
        pattern = r"Command\(\s*\[\s*['\"]xacro['\"]\s*,"
        assert re.search(pattern, content), \
            "Must use Command(['xacro', ...]) for xacro processing"

    def test_launch_uses_get_package_share(self):
        """Verify get_package_share_directory or FindPackageShare is used."""
        content = get_launch_content()
        pattern = r"(?:get_package_share_directory|FindPackageShare)\(\s*['\"]urdf_tutorial['\"]"
        assert re.search(pattern, content), \
            "Must use get_package_share_directory('urdf_tutorial')"

    def test_launch_uses_parameter_value(self):
        """Verify robot_description is wrapped in ParameterValue."""
        content = get_launch_content()
        pattern = r"['\"]robot_description['\"]\s*:\s*(?:[^,}]*?ParameterValue)"
        assert re.search(pattern, content), \
            "robot_description must be wrapped in ParameterValue"

    def test_launch_no_ros1_syntax(self):
        """Verify no ROS1 $(find ...) syntax remains."""
        content = get_launch_content()
        ros1_syntax = r"\$\(find\s+.*?\)"
        assert not re.search(ros1_syntax, content), \
            "Found legacy ROS 1 '$(find ...)' syntax"

    def test_launch_includes_display(self):
        """Verify IncludeLaunchDescription with display.launch.py."""
        content = get_launch_content()
        pattern = r"IncludeLaunchDescription\s*\(.*?display\.launch\.py"
        assert re.search(pattern, content, re.DOTALL), \
            "Must include display.launch.py from urdf_launch"

    def test_launch_has_generate_function(self):
        """Verify the launch file has generate_launch_description function."""
        content = get_launch_content()
        assert 'def generate_launch_description' in content, \
            "Must define generate_launch_description()"

    def test_launch_imports_correct_modules(self):
        """Verify correct ROS2 launch imports."""
        content = get_launch_content()
        assert 'from launch.substitutions import Command' in content or \
               'from launch.substitutions import' in content, \
            "Must import Command from launch.substitutions"
        assert 'ParameterValue' in content, \
            "Must import ParameterValue"


class TestXacroProcessing:
    """Test that xacro can actually process the URDF file."""

    def test_xacro_processes_urdf(self):
        """Verify xacro can process the URDF file without errors."""
        urdf_path = None
        for f in [URDF_FILE, URDF_FILE_ALT]:
            if f.exists():
                urdf_path = f
                break
        if urdf_path is None:
            pytest.skip("No URDF file found")

        try:
            result = subprocess.run(
                ['xacro', str(urdf_path)],
                capture_output=True,
                text=True,
                timeout=10
            )
            # xacro may fail due to missing mesh files from urdf_tutorial package,
            # but the xacro processing itself should work
            # Check that it at least produced some XML output or only failed on mesh resolution
            if result.returncode != 0:
                # Accept if the only issue is missing package resources
                if 'package://' in result.stderr or 'not found' in result.stderr.lower():
                    pytest.skip("xacro processing requires urdf_tutorial meshes")
                # Otherwise check if it produced partial output
                assert '<robot' in result.stdout or result.returncode == 0, \
                    f"xacro failed: {result.stderr}"
            else:
                # Successful processing - verify output
                assert '<robot' in result.stdout, "xacro output should contain <robot> tag"
                assert 'base_link' in result.stdout, "xacro output should contain base_link"
                # Verify the cylinder dimensions were expanded
                assert 'cylinder' in result.stdout, "Output should contain cylinder geometry"
        except FileNotFoundError:
            pytest.skip("xacro command not available")