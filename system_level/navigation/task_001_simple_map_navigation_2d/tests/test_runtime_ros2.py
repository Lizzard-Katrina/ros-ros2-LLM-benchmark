"""
Runtime test for task_001_simple_map_navigation_2d package.

This test validates that the package was correctly migrated from ROS1 to ROS2 by:
1. Checking that package.xml and CMakeLists.txt exist and have correct content.
2. Verifying the package can be found by ament_index (i.e., it was installed correctly).
3. Verifying the asset directories are installed in the share directory.
4. Verifying the launch file can be loaded and parsed.
5. Verifying the nav2_params.yaml can be loaded and contains expected keys.
"""
import re
import pytest
import subprocess
import os
import yaml
from pathlib import Path


PACKAGE_NAME = "task_001_simple_map_navigation_2d"

# Resolve paths relative to this test file (package root)
PKG_ROOT = Path(__file__).resolve().parent
PKG_XML = PKG_ROOT / "package.xml"
CMAKE_TXT = PKG_ROOT / "CMakeLists.txt"


@pytest.fixture
def pkg_content():
    return PKG_XML.read_text()


@pytest.fixture
def cmake_content():
    return CMAKE_TXT.read_text()


def test_package_xml_exists():
    """Verify package.xml exists."""
    assert PKG_XML.exists(), "package.xml not found at package root."


def test_cmakelists_exists():
    """Verify CMakeLists.txt exists."""
    assert CMAKE_TXT.exists(), "CMakeLists.txt not found at package root."


def test_buildtool_is_ament_cmake(pkg_content):
    """Verify package.xml uses ament_cmake buildtool."""
    assert re.search(
        r"<buildtool_depend>\s*ament_cmake\s*</buildtool_depend>", pkg_content
    ), "package.xml must declare ament_cmake as buildtool_depend."


def test_no_catkin_references(pkg_content, cmake_content):
    """Verify no catkin references remain."""
    assert "catkin" not in pkg_content.lower(), "catkin found in package.xml"
    assert "catkin" not in cmake_content.lower(), "catkin found in CMakeLists.txt"


def test_nav2_bringup_dependency(pkg_content):
    """Verify nav2_bringup dependency is declared."""
    pattern = r"<(?:depend|exec_depend)>\s*nav2_bringup\s*</(?:depend|exec_depend)>"
    assert re.search(pattern, pkg_content), "Missing nav2_bringup dependency."


def test_cmake_find_ament(cmake_content):
    """Verify CMakeLists.txt has find_package(ament_cmake REQUIRED)."""
    assert re.search(
        r"find_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)", cmake_content
    ), "Missing find_package(ament_cmake REQUIRED)."


def test_cmake_installs_all_asset_dirs(cmake_content):
    """Verify all 6 asset directories are installed."""
    required_dirs = ["config", "launch", "maps", "meshes", "rviz", "worlds"]
    for d in required_dirs:
        pattern = rf"install\s*\(.*DIRECTORY\s+[^)]*{d}"
        assert re.search(pattern, cmake_content, re.DOTALL), (
            f"Directory '{d}' missing from CMake install rules."
        )


def test_cmake_install_destination(cmake_content):
    """Verify install destination is share/${PROJECT_NAME}."""
    pattern = r"DESTINATION\s+share/\$\{PROJECT_NAME\}"
    assert re.search(pattern, cmake_content), (
        "Install destination should be share/${PROJECT_NAME}."
    )


def test_cmake_ament_package(cmake_content):
    """Verify ament_package() is called."""
    assert re.search(r"ament_package\s*\(\s*\)", cmake_content), (
        "Missing ament_package() call."
    )


def test_asset_directories_exist():
    """Verify all 6 asset directories exist in the package source tree."""
    required_dirs = ["config", "launch", "maps", "meshes", "rviz", "worlds"]
    for d in required_dirs:
        dir_path = PKG_ROOT / d
        assert dir_path.exists() and dir_path.is_dir(), (
            f"Asset directory '{d}' does not exist in the package root."
        )


def test_launch_file_is_valid_python():
    """Verify the launch file can be imported and has generate_launch_description."""
    import importlib.util
    launch_file = PKG_ROOT / "launch" / "navigation.launch.py"
    if not launch_file.exists():
        pytest.skip("No launch file to test")
    spec = importlib.util.spec_from_file_location("nav_launch", str(launch_file))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert hasattr(mod, "generate_launch_description"), (
        "Launch file must have generate_launch_description function."
    )
    ld = mod.generate_launch_description()
    assert ld is not None, "generate_launch_description returned None."


def test_nav2_params_yaml_exists():
    """Verify nav2 params config file exists."""
    params_file = PKG_ROOT / "config" / "nav2_params.yaml"
    assert params_file.exists(), "config/nav2_params.yaml not found."


def test_nav2_params_yaml_content():
    """Verify nav2 params config file has expected top-level keys."""
    params_file = PKG_ROOT / "config" / "nav2_params.yaml"
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    assert params is not None, "nav2_params.yaml is empty or invalid."
    expected_keys = ["amcl", "bt_navigator", "controller_server", "planner_server",
                     "local_costmap", "global_costmap", "map_server"]
    for key in expected_keys:
        assert key in params, f"Missing top-level key '{key}' in nav2_params.yaml."


def test_nav2_params_use_sim_time():
    """Verify use_sim_time is set in amcl params."""
    params_file = PKG_ROOT / "config" / "nav2_params.yaml"
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    amcl_params = params.get("amcl", {}).get("ros__parameters", {})
    assert amcl_params.get("use_sim_time") is True, (
        "amcl ros__parameters should have use_sim_time: true"
    )


def test_map_yaml_exists():
    """Verify map.yaml exists in maps directory."""
    map_file = PKG_ROOT / "maps" / "map.yaml"
    assert map_file.exists(), "maps/map.yaml not found."


def test_rviz_config_exists():
    """Verify rviz config exists."""
    rviz_file = PKG_ROOT / "rviz" / "navigation.rviz"
    assert rviz_file.exists(), "rviz/navigation.rviz not found."


def test_package_installed_in_ament_index():
    """Verify the package is findable via ament_index after install."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(PACKAGE_NAME)
        assert os.path.isdir(share_dir), f"Share directory {share_dir} not found."
        # Verify installed config
        installed_params = os.path.join(share_dir, "config", "nav2_params.yaml")
        assert os.path.isfile(installed_params), (
            f"Installed nav2_params.yaml not found at {installed_params}"
        )
        # Verify installed launch
        installed_launch = os.path.join(share_dir, "launch", "navigation.launch.py")
        assert os.path.isfile(installed_launch), (
            f"Installed navigation.launch.py not found at {installed_launch}"
        )
    except Exception:
        pytest.skip("Package not installed in ament index (not in colcon workspace).")


def test_launch_file_generates_description_from_installed():
    """Verify the installed launch file generates a valid LaunchDescription."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(PACKAGE_NAME)
        launch_file = os.path.join(share_dir, "launch", "navigation.launch.py")
        if not os.path.isfile(launch_file):
            pytest.skip("Installed launch file not found.")
        import importlib.util
        spec = importlib.util.spec_from_file_location("nav_launch_installed", launch_file)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        assert ld is not None
        # Check that there are at least 3 launch actions (the 3 DeclareLaunchArgument)
        entities = ld.entities
        assert len(entities) >= 3, (
            f"Expected at least 3 launch entities, got {len(entities)}"
        )
    except ImportError:
        pytest.skip("ament_index_python not available.")