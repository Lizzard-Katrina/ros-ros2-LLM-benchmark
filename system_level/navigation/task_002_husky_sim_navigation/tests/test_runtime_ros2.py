"""
Runtime test for task_002_husky_sim_navigation.

This test validates:
1. The package.xml is well-formed with correct ROS2 dependencies.
2. The CMakeLists.txt installs required directories.
3. The launch file (move_base.launch) is valid Python launch and generates
   a proper LaunchDescription with the expected arguments and conditions.
4. Cross-file consistency between launch and package.xml.
"""
import os
import re
import sys
import types
import pytest
from pathlib import Path

# Determine package root (where this test file lives)
PKG_ROOT = Path(__file__).resolve().parent


def _read(relpath):
    p = PKG_ROOT / relpath
    if p.exists():
        return p.read_text()
    return ""


# ---- Static / structural checks ----

class TestPackageXml:
    def test_format_3(self):
        content = _read("package.xml")
        assert re.search(r'<package\s+format=["\']3["\']>', content)

    def test_ament_cmake(self):
        content = _read("package.xml")
        assert re.search(r'<buildtool_depend>\s*ament_cmake\s*</buildtool_depend>', content)

    def test_no_catkin(self):
        content = _read("package.xml")
        assert "catkin" not in content.lower()

    def test_nav2_bringup(self):
        content = _read("package.xml")
        assert re.search(r'<(?:depend|exec_depend)>\s*nav2_bringup\s*</', content)

    def test_slam_toolbox(self):
        content = _read("package.xml")
        assert re.search(r'<(?:depend|exec_depend)>\s*slam_toolbox\s*</', content)

    def test_no_move_base(self):
        content = _read("package.xml")
        assert not re.search(r'<(?:depend|run_depend)>\s*move_base\s*</', content)


class TestCMakeLists:
    def test_install_config(self):
        content = _read("CMakeLists.txt")
        assert re.search(r'install\s*\(\s*DIRECTORY\s+[^)]*\bconfig\b', content, re.DOTALL | re.IGNORECASE)

    def test_install_launch(self):
        content = _read("CMakeLists.txt")
        assert re.search(r'install\s*\(\s*DIRECTORY\s+[^)]*\blaunch\b', content, re.DOTALL | re.IGNORECASE)

    def test_install_maps(self):
        content = _read("CMakeLists.txt")
        assert re.search(r'install\s*\(\s*DIRECTORY\s+[^)]*\bmaps\b', content, re.DOTALL | re.IGNORECASE)

    def test_ament_package(self):
        content = _read("CMakeLists.txt")
        assert re.search(r'ament_package\s*\(\s*\)', content)

    def test_no_catkin_package(self):
        content = _read("CMakeLists.txt")
        assert "catkin_package" not in content


class TestLaunchFile:
    """Test the move_base.launch file (which is actually Python)."""

    def test_file_exists(self):
        assert (PKG_ROOT / "move_base.launch").exists()

    def test_get_package_share_directory(self):
        content = _read("move_base.launch")
        assert 'get_package_share_directory' in content

    def test_no_ros1_find(self):
        content = _read("move_base.launch")
        assert "$(find" not in content

    def test_no_static_map_arg(self):
        content = _read("move_base.launch")
        assert re.search(r'DeclareLaunchArgument\s*\(\s*[\'"]no_static_map[\'"]', content)

    def test_conditional_logic(self):
        content = _read("move_base.launch")
        assert re.search(r'(?:IfCondition|UnlessCondition|PythonExpression)', content)


class TestCrossFileConsistency:
    def test_nav2_lifecycle_manager_sync(self):
        pkg = _read("package.xml")
        launch = _read("move_base.launch")
        if 'nav2_lifecycle_manager' in launch:
            assert 'nav2_lifecycle_manager' in pkg

    def test_nav2_bringup_sync(self):
        pkg = _read("package.xml")
        launch = _read("move_base.launch")
        if 'nav2_bringup' in launch:
            assert 'nav2_bringup' in pkg

    def test_slam_toolbox_sync(self):
        pkg = _read("package.xml")
        launch = _read("move_base.launch")
        if 'slam_toolbox' in launch:
            assert 'slam_toolbox' in pkg


class TestLaunchRuntime:
    """
    Actually import and execute the launch file's generate_launch_description()
    to verify it produces a valid LaunchDescription with the expected entities.
    """

    def _load_launch_module(self, filepath):
        """Load a Python file that doesn't have a .py extension as a module."""
        source = filepath.read_text()
        mod = types.ModuleType("move_base_launch")
        mod.__file__ = str(filepath)
        code = compile(source, str(filepath), 'exec')
        exec(code, mod.__dict__)
        return mod

    def test_generate_launch_description(self):
        """Import the actual move_base.launch file and call generate_launch_description."""
        launch_file = PKG_ROOT / "move_base.launch"
        assert launch_file.exists(), "move_base.launch not found"

        # We need to mock get_package_share_directory since the package may not
        # be installed yet. We'll patch it to return the package root.
        import ament_index_python.packages as aip
        original_gpsd = aip.get_package_share_directory

        def mock_gpsd(pkg_name):
            if pkg_name == 'task_002_husky_sim_navigation':
                return str(PKG_ROOT)
            # For nav2_bringup, try the real one; if not installed, return a dummy
            try:
                return original_gpsd(pkg_name)
            except Exception:
                # Return a temp dir so os.path.join doesn't fail
                import tempfile
                return tempfile.mkdtemp()

        aip.get_package_share_directory = mock_gpsd
        try:
            mod = self._load_launch_module(launch_file)
            assert hasattr(mod, 'generate_launch_description'), \
                "move_base.launch must define generate_launch_description()"

            ld = mod.generate_launch_description()

            from launch import LaunchDescription
            assert isinstance(ld, LaunchDescription), \
                "generate_launch_description() must return a LaunchDescription"

            # Check that we have entities
            entities = ld.entities
            assert len(entities) > 0, "LaunchDescription should have entities"

            # Check that DeclareLaunchArgument for no_static_map is present
            from launch.actions import DeclareLaunchArgument
            declared_args = [
                e for e in entities
                if isinstance(e, DeclareLaunchArgument)
            ]
            arg_names = [a.name for a in declared_args]
            assert 'no_static_map' in arg_names, \
                "Expected 'no_static_map' DeclareLaunchArgument in LaunchDescription"

            # Check that there are GroupActions with conditions
            from launch.actions import GroupAction
            groups = [e for e in entities if isinstance(e, GroupAction)]
            assert len(groups) >= 2, \
                "Expected at least 2 GroupActions (static map and SLAM modes)"

            # Verify at least one group has a condition
            conditioned = [g for g in groups if g.condition is not None]
            assert len(conditioned) >= 1, \
                "Expected at least one GroupAction with a condition (IfCondition/UnlessCondition)"

        finally:
            aip.get_package_share_directory = original_gpsd

    def test_launch_py_file_also_works(self):
        """Also verify the launch/move_base.launch.py file works identically."""
        launch_py_file = PKG_ROOT / "launch" / "move_base.launch.py"
        if not launch_py_file.exists():
            pytest.skip("launch/move_base.launch.py not found")

        import importlib.util
        import ament_index_python.packages as aip
        original_gpsd = aip.get_package_share_directory

        def mock_gpsd(pkg_name):
            if pkg_name == 'task_002_husky_sim_navigation':
                return str(PKG_ROOT)
            try:
                return original_gpsd(pkg_name)
            except Exception:
                import tempfile
                return tempfile.mkdtemp()

        aip.get_package_share_directory = mock_gpsd
        try:
            spec = importlib.util.spec_from_file_location(
                "move_base_launch_py", str(launch_py_file))
            assert spec is not None, "Could not create spec for launch/move_base.launch.py"
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)

            assert hasattr(mod, 'generate_launch_description')
            ld = mod.generate_launch_description()

            from launch import LaunchDescription
            assert isinstance(ld, LaunchDescription)
            assert len(ld.entities) > 0
        finally:
            aip.get_package_share_directory = original_gpsd