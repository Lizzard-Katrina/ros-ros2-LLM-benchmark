import re
import pytest
from pathlib import Path

# File paths based on standard ROS2 package structure
BURGER_URDF = Path(__file__).resolve().parents[1] / "turtlebot3_burger_cam.urdf"
WAFFLE_URDF = Path(__file__).resolve().parents[1] / "turtlebot3_waffle.urdf"
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "turtlebot3_world.launch.py"

def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

# --- Burger URDF Tests ---

def test_burger_base_link_mesh():
    """Verify Burger uses the correct base mesh and scale."""
    content = read_file(BURGER_URDF)
    pattern = r'<mesh\s+filename="package://turtlebot3_gazebo/models/turtlebot3_common/meshes/bases/burger_base\.stl"\s+scale="0\.001\s+0\.001\s+0\.001"'
    assert re.search(pattern, content), "Burger URDF missing correct base mesh path or scale."

def test_burger_collision_geometry():
    """Verify Burger collision box is sized for the small cylindrical chassis."""
    content = read_file(BURGER_URDF)
    # Check for the specific box size used for Burger variant
    pattern = r'<box\s+size="0\.140?\s+0\.140?\s+0\.143?"'
    assert re.search(pattern, content), "Burger collision box dimensions are incorrect or missing."

def test_burger_inertial_properties():
    """Verify Burger mass is approximately 0.825kg."""
    content = read_file(BURGER_URDF)
    assert re.search(r'<mass\s+value="8\.257\d+e-01"', content), "Burger mass value is incorrect."

# --- Waffle URDF Tests ---

def test_waffle_dual_caster_structure():
    """Verify Waffle has both left and right back casters (unlike Burger)."""
    content = read_file(WAFFLE_URDF)
    assert "caster_back_right_link" in content, "Waffle missing caster_back_right_link."
    assert "caster_back_left_link" in content, "Waffle missing caster_back_left_link."

def test_waffle_base_link_mesh():
    """Verify Waffle uses the correct base mesh (Waffle vs Burger check)."""
    content = read_file(WAFFLE_URDF)
    assert "meshes/bases/waffle_base.stl" in content, "Waffle URDF using wrong base mesh."

# --- Launch File Tests ---

def test_launch_uses_modern_gz_sim():
    """Verify the launch file uses ros_gz_sim (New Gazebo) instead of gazebo_ros (Classic)."""
    content = read_file(LAUNCH_FILE)
    # Ensure ros_gz_sim package is retrieved and used in launch source
    assert "ros_gz_sim" in content, "Launch file should use ros_gz_sim package."
    assert "gz_sim.launch.py" in content, "Launch file missing modern gz_sim.launch.py reference."

def test_launch_server_client_separation():
    """Verify separation of gzserver (headless) and gzclient (gui) logic."""
    content = read_file(LAUNCH_FILE)
    # Server needs -s (server) and -r (run)
    server_pattern = r"'-r\s+-s\s+-v2\s*',\s*world"
    # Client needs -g (gui)
    client_pattern = r"'-g\s+-v2\s*'"
    assert re.search(server_pattern, content), "GzServer command missing -r -s arguments."
    assert re.search(client_pattern, content), "GzClient command missing -g argument."

def test_launch_environment_resource_path():
    """Verify GZ_SIM_RESOURCE_PATH is set for mesh loading."""
    content = read_file(LAUNCH_FILE)
    assert "GZ_SIM_RESOURCE_PATH" in content, "Missing GZ_SIM_RESOURCE_PATH environment variable."
    assert "AppendEnvironmentVariable" in content, "Environment variable should be appended."

def test_no_legacy_gazebo_nodes():
    """Ensure no legacy gazebo_ros nodes are being instantiated."""
    content = read_file(LAUNCH_FILE)
    assert "gazebo_ros" not in content, "Legacy gazebo_ros found. Task requires modern ros_gz_sim."