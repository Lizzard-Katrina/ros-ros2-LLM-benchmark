
import re
import pytest
from pathlib import Path

# Define file paths using the standardized resolve pattern
XACRO_FILE = Path(__file__).resolve().parents[1] / "husky.gazebo.xacro"
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "husky_empty_world.launch"

# -------------------------
# Helper function (Updated)
# -------------------------
def get_content(file_path: Path) -> str:
    """
    Read and return the content of the specified path
    """
    assert file_path.exists(), f"File not found: {file_path.name}"
    return file_path.read_text()


# -------------------------
# Test Group 0: File existence
# -------------------------
def test_translated_files_exist():
    """
    Both source files must be translated and present
    """
    assert XACRO_FILE.exists(), "Translated file missing: husky.gazebo.xacro"
    assert LAUNCH_FILE.exists(), "Translated file missing: husky_empty_world.launch"


# -------------------------
# Test Group 1: ROS1 artifact removal
# -------------------------
def test_no_ros1_artifacts_in_launch():
    """
    ROS1 launch-specific APIs should not appear in translated ROS2 launch
    """
    code = get_content(LAUNCH_FILE)
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
    code = get_content(XACRO_FILE)
    required_keywords = ["robot", "link", "joint"]
    for kw in required_keywords:
        assert kw in code, f"robot_description keyword missing in xacro: {kw}"


def test_robot_description_consumed_in_launch():
    """
    Launch file uses robot_description parameter
    """
    code = get_content(LAUNCH_FILE)
    assert "robot_description" in code, "Launch file does not reference robot_description"


# -------------------------
# Test Group 3: IMU/GPS publishers
# -------------------------
def test_imu_interface_present():
    """
    IMU plugin/interface exists in translated URDF
    """
    code = get_content(XACRO_FILE)
    imu_keywords = ["imu", "inertial", "sensor", "imu_controller"]
    found = any(kw in code for kw in imu_keywords)
    assert found, "IMU interface missing in xacro"


def test_gps_interface_present():
    """
    GPS/NavSat plugin/interface exists in translated URDF
    """
    code = get_content(XACRO_FILE)
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
    code = get_content(XACRO_FILE)
    rate_keywords = ["updateRate", "frequency", "hz", "publish_rate"]
    found = any(kw in code for kw in rate_keywords)
    assert found, "Sensor update rate semantics missing in xacro"
