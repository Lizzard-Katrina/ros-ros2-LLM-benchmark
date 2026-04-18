import re
import pytest
from pathlib import Path

# Paths to the hollowed files
PKG_XML = Path(__file__).resolve().parents[1] / "package.xml"
CMAKE_TXT = Path(__file__).resolve().parents[1] / "CMakeLists.txt"

@pytest.fixture
def pkg_content():
    return PKG_XML.read_text()

@pytest.fixture
def cmake_content():
    return CMAKE_TXT.read_text()

def test_pkg_build_system_transition(pkg_content):
    """Verify package.xml uses ament_cmake and NOT catkin."""
    assert re.search(r"<buildtool_depend>\s*ament_cmake\s*</buildtool_depend>", pkg_content), \
        "package.xml must declare ament_cmake as the buildtool_depend."
    assert "catkin" not in pkg_content.lower(), "Legacy 'catkin' dependency found in package.xml."

def test_pkg_nav2_dependency(pkg_content):
    """Verify that nav2_bringup is added as a dependency for a navigation project."""
    # Matches <depend>nav2_bringup</depend> or <exec_depend>
    pattern = r"<(?:depend|exec_depend)>\s*nav2_bringup\s*</(?:depend|exec_depend)>"
    assert re.search(pattern, pkg_content), \
        "Missing 'nav2_bringup' dependency in package.xml, which is required for this nav stack."

def test_cmake_ament_init(cmake_content):
    """Verify CMakeLists.txt initializes ament_cmake correctly."""
    assert re.search(r"find_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)", cmake_content), \
        "CMakeLists.txt must find_package ament_cmake."

def test_cmake_asset_installation_coverage(cmake_content):
    """Verify all 6 physical asset directories are included in the install command."""
    required_dirs = ['config', 'launch', 'maps', 'meshes', 'rviz', 'worlds']
    for d in required_dirs:
        # Matches 'install' followed by the directory name somewhere before the next command
        pattern = rf"install\s*\(.*DIRECTORY\s+[^)]*{d}"
        assert re.search(pattern, cmake_content, re.DOTALL), \
            f"Directory '{d}' is missing from the CMake installation rules."

def test_cmake_install_destination(cmake_content):
    """Verify assets are mapped to the correct ROS 2 share destination."""
    pattern = r"DESTINATION\s+share/\$\{PROJECT_NAME\}"
    assert re.search(pattern, cmake_content), \
        "Incorrect install destination. Should be share/${PROJECT_NAME} for ROS 2."

def test_cmake_package_registration(cmake_content):
    """Verify the ament_package() macro is called at the end."""
    assert re.search(r"ament_package\s*\(\s*\)", cmake_content), \
        "CMakeLists.txt is missing ament_package() call to register the package."

def test_cross_file_dependency_sync(pkg_content, cmake_content):
    """Verify synchronization: dependencies in XML should be find_package'd in CMake."""
    # Extract dependencies from XML (simple capture)
    xml_deps = re.findall(r"<(?:depend|build_depend)>\s*([^<]+)\s*</", pkg_content)
    # Filter out ament_cmake as it's a buildtool
    user_deps = [d.strip() for d in xml_deps if 'ament_cmake' not in d]
    
    for dep in user_deps:
        # Check if each XML dependency is also find_package'd in CMake
        pattern = rf"find_package\s*\(\s*{dep}"
        assert re.search(pattern, cmake_content), \
            f"Dependency '{dep}' declared in package.xml but not found in CMakeLists.txt."
