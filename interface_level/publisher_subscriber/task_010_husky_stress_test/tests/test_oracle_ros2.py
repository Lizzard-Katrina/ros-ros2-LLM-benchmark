# src/task_010/test/test_oracle_ros2.py
import pytest
from pathlib import Path

# -------------------------
# Helper function
# -------------------------
def load_translated_file(filename: str) -> str:
    """
    Load the translated ROS2 file from submission directory
    """
    task_root_dir = Path(__file__).resolve().parent.parent
    file_path = Path(__file__).parent.parent / filename
    assert file_path.exists(), f"Translated ROS2 file not found: {filename}"
    return file_path.read_text()


# -------------------------
# Test Group 0: File existence
# -------------------------
def test_translated_files_exist():
    """
    Both source files must be translated and present
    """
    for f in ["husky.gazebo.xacro", "husky_empty_world.launch"]:
        file_path = Path(__file__).parent.parent / f
        assert file_path.exists(), f"Translated file missing: {f}"


# -------------------------
# Test Group 1: ROS1 artifact removal
# -------------------------
def test_no_ros1_artifacts_in_launch():
    """
    ROS1 launch-specific APIs should not appear in translated ROS2 launch
    """
    code = load_translated_file("husky_empty_world.launch")
    forbidden_patterns = ["<node pkg=", "$(find", "rostopic", "rosparam", "launch"]
    for pat in forbidden_patterns:
        assert pat not in code, f"ROS1 artifact found in launch file: {pat}"


# -------------------------
# Test Group 2: robot_description interface
# -------------------------
def test_robot_description_defined():
    """
    ROS2 Xacro/URDF translation preserves robot_description interface
    """
    code = load_translated_file("husky.gazebo.xacro")
    required_keywords = ["robot", "link", "joint"]
    for kw in required_keywords:
        assert kw in code, f"robot_description keyword missing in xacro: {kw}"


def test_robot_description_consumed_in_launch():
    """
    Launch file uses robot_description parameter
    """
    code = load_translated_file("husky_empty_world.launch")
    assert "robot_description" in code, "Launch file does not reference robot_description"


# -------------------------
# Test Group 3: IMU/GPS publishers
# -------------------------
def test_imu_interface_present():
    """
    IMU plugin/interface exists in translated URDF
    """
    code = load_translated_file("husky.gazebo.xacro")
    imu_keywords = ["imu", "inertial", "sensor", "imu_controller"]
    found = any(kw in code for kw in imu_keywords)
    assert found, "IMU interface missing in xacro"


def test_gps_interface_present():
    """
    GPS/NavSat plugin/interface exists in translated URDF
    """
    code = load_translated_file("husky.gazebo.xacro")
    gps_keywords = ["gps", "navsat", "fix", "gps_controller"]
    found = any(kw in code for kw in gps_keywords)
    assert found, "GPS/NavSat interface missing in xacro"


# -------------------------
# Test Group 4: Sensor update rate / high-frequency semantics
# -------------------------
def test_sensor_update_rate_semantics():
    """
    IMU or GPS update rates preserved
    """
    code = load_translated_file("husky.gazebo.xacro")
    rate_keywords = ["updateRate", "frequency", "hz", "publish_rate"]
    found = any(kw in code for kw in rate_keywords)
    assert found, "Sensor update rate semantics missing in xacro"
