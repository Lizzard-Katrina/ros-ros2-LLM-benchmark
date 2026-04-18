import re
from pathlib import Path

# File paths based on the Husky package structure
PKG_XML = Path(__file__).resolve().parents[1] / "package.xml"
CMAKE_FILE = Path(__file__).resolve().parents[1] / "CMakeLists.txt"
LAUNCH_FILE = Path(__file__).resolve().parents[1] /"move_base.launch"

def get_content(path):
    return path.read_text() if path.exists() else ""

# --- 1. Build System Modernization ---

def test_pkg_format_and_buildtool():
    """Verify transition from catkin to ament_cmake."""
    content = get_content(PKG_XML)
    # Check for format 3 and ament_cmake buildtool
    assert re.search(r'<package\s+format=["\']3["\']>', content), "package.xml must use format 3."
    assert re.search(r'<buildtool_depend>\s*ament_cmake\s*</buildtool_depend>', content), \
        "Buildtool must be transitioned from catkin to ament_cmake."
    assert "catkin" not in content.lower(), "Legacy catkin references should be removed."

# --- 2. Functional Dependency Mapping ---

def test_nav_stack_replacement():
    """Verify move_base/gmapping are replaced by Nav2/SlamToolbox."""
    content = get_content(PKG_XML)
    # Check for presence of modern functional equivalents
    assert re.search(r'<(?:depend|exec_depend)>\s*nav2_bringup\s*</', content), \
        "Missing nav2_bringup dependency to replace move_base functionality."
    assert re.search(r'<(?:depend|exec_depend)>\s*slam_toolbox\s*</', content), \
        "Missing slam_toolbox dependency to replace gmapping functionality."
    # Ensure legacy nodes are absent
    assert not re.search(r'<(?:depend|run_depend)>\s*move_base\s*</', content), \
        "Legacy move_base dependency should not be present in ROS2 manifest."

# --- 3. Resource Installation Rules ---

def test_cmake_asset_installation():
    """Verify that all 3 physical directories (config, launch, maps) are installed."""
    content = get_content(CMAKE_FILE)
    # Match install(DIRECTORY ... with config, launch, and maps
    # Uses non-capturing groups to allow any order within the DIRECTORY command
    for folder in ['config', 'launch', 'maps']:
        pattern = rf'install\s*\(\s*DIRECTORY\s+[^)]*\b{folder}\b'
        assert re.search(pattern, content, re.IGNORECASE | re.DOTALL), \
            f"The '{folder}' directory is not marked for installation in CMakeLists.txt."

# --- 4. Package Registration ---

def test_cmake_ament_registration():
    """Verify standard ROS2 package registration."""
    content = get_content(CMAKE_FILE)
    assert re.search(r'ament_package\s*\(\s*\)', content), \
        "CMakeLists.txt is missing the required ament_package() call."
    assert "catkin_package" not in content, "Legacy catkin_package() macro must be removed."

# --- 5. Path Resolution Logic ---

def test_launch_path_resolution():
    """Verify Python launch uses ROS2 resource location methods."""
    content = get_content(LAUNCH_FILE)
    # Search for the conceptual use of get_package_share_directory
    assert 'get_package_share_directory' in content
    assert "$(find" not in content, "Legacy ROS1 $(find ...) syntax is not allowed in Python launch files."

# --- 6. Functional Parity: Conditional Logic ---

def test_launch_conditional_logic():
    """Verify the 'no_static_map' logic from original XML is ported to Python."""
    content = get_content(LAUNCH_FILE)
    # Verify the argument exists
    assert re.search(r'DeclareLaunchArgument\s*\(\s*[\'"]no_static_map[\'"]', content), \
        "The 'no_static_map' argument declaration is missing in the new launch file."
    # Verify conditional execution (IfCondition or UnlessCondition)
    assert re.search(r'(?:IfCondition|UnlessCondition|PythonExpression)', content), \
        "The conditional logic from the original XML (if/unless) was not ported to Python Launch conditions."

# --- 7. Cross-File Consistency ---

def test_system_dependency_sync():
    """Ensure launch-invoked packages are declared in the manifest."""
    pkg_content = get_content(PKG_XML)
    launch_content = get_content(LAUNCH_FILE)
    
    # Identify common Nav2 packages in launch and check XML
    for nav_pkg in ['nav2_bringup', 'nav2_lifecycle_manager', 'slam_toolbox']:
        if nav_pkg in launch_content:
            assert nav_pkg in pkg_content, \
                f"Package '{nav_pkg}' is used in launch logic but missing from package.xml."
