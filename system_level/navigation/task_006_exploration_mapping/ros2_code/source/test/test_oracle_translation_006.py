import re
from pathlib import Path

# Paths to the translated ROS 2 files
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "mapping.launch"
YAML_FILE = Path(__file__).resolve().parents[1] / "costmap_common_params.yaml"


def get_content(f):
    return Path(f).read_text() if Path(f).exists() else ""


def test_hole1_slam_node_translation():
    """
    Verify Hole 1: Mapping Node Translation.
    """
    content = get_content(LAUNCH_FILE)

    assert 'exec=' in content, "ROS 2 nodes must use the 'exec' attribute instead of 'type'."

    valid_ros2_slam = ["slam_toolbox", "async_slam_toolbox", "nav2_map_server"]
    assert any(term in content for term in valid_ros2_slam), \
        "Mapping semantics lost. No ROS 2 SLAM/Map components found."

    assert 'pkg="gmapping"' not in content, \
        "ROS 1 residue found: pkg='gmapping' should be migrated."
    assert 'type="slam_gmapping"' not in content, \
        "ROS 1 residue found: type='slam_gmapping' should be migrated."


def test_hole2_costmap_sensor_translation():
    """
    Verify Hole 2: Observation Source Translation.
    """
    content = get_content(YAML_FILE)

    assert re.search(r"topic:\s+/?scan", content), \
        "Sensor topic '/scan' missing or incorrectly translated."

    assert "marking: true" in content.lower(), \
        "Mapping logic 'marking' is missing in the costmap translation."
    assert "clearing: true" in content.lower(), \
        "Mapping logic 'clearing' is missing in the costmap translation."

    assert "footprint:" in content and "0.175" in content, \
        "Robot footprint coordinates were lost or corrupted."


def test_rviz_migration_consistency():
    """
    Verify that peripheral components (RViz) are also migrated to ROS 2 versions.
    """
    content = get_content(LAUNCH_FILE)
    assert "rviz2" in content, "RViz was not migrated to rviz2."
    assert 'pkg="rviz"' not in content, "Found ROS 1 'rviz' package name residue."