import re
from pathlib import Path
import pytest

BASE_DIR = Path(__file__).resolve().parents[1]
CPP_FILE = BASE_DIR / "robot_hardware_interface_node.cpp"
YAML_FILE = BASE_DIR / "controllers.yaml"

def get_content(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

def test_architecture_transformation():
    """Verify conversion from standalone Node to SystemInterface plugin."""
    content = get_content(CPP_FILE)
    assert re.search(r"public\s+hardware_interface::SystemInterface", content), \
        "Class must inherit from hardware_interface::SystemInterface."
    assert not re.search(r"int\s+main\s*\(", content), \
        "Hardware Interface must be a plugin; standalone main() detected."
    assert "PLUGINLIB_EXPORT_CLASS" in content, "Missing plugin export macro."

def test_lifecycle_methods_presence():
    """Verify implementation of ROS 2 Control lifecycle callbacks."""
    content = get_content(CPP_FILE)
    assert "on_init" in content
    assert "export_state_interfaces" in content
    assert "export_command_interfaces" in content
    assert "CallbackReturn::SUCCESS" in content

def test_interface_matching_logic():
    """Verify that interfaces (Velocity/Position) match system requirements."""
    content = get_content(CPP_FILE)
    # Controller requires Velocity command and Position/Velocity state
    assert re.search(r"HW_IF_POSITION", content)
    assert re.search(r"HW_IF_VELOCITY", content)
    assert "left_wheel_joint" in content and "right_wheel_joint" in content

def test_ros2_read_write_signatures():
    """Verify read/write methods use ROS 2 signatures (Time, Duration)."""
    content = get_content(CPP_FILE)
    read_pattern = r"read\s*\(\s*const\s+rclcpp::Time\s*&.*,\s*const\s+rclcpp::Duration\s*&.*\)"
    write_pattern = r"write\s*\(\s*const\s+rclcpp::Time\s*&.*,\s*const\s+rclcpp::Duration\s*&.*\)"
    assert re.search(read_pattern, content), "Incorrect read() signature."
    assert re.search(write_pattern, content), "Incorrect write() signature."

def test_yaml_structure_and_sync():
    """Verify YAML configuration matches CPP interface naming and types."""
    content = get_content(YAML_FILE)
    # Check ROS 2 specific nesting
    assert "controller_manager:" in content
    assert "ros__parameters:" in content
    # Check for DiffDriveController specific sync
    assert "diff_drive_controller/DiffDriveController" in content
    assert "left_wheel_joint" in content and "right_wheel_joint" in content
    assert re.search(r"command_interfaces:.*velocity", content, re.DOTALL)

def test_legacy_cleanup():
    """Ensure no ROS 1 components remain."""
    content = get_content(CPP_FILE)
    assert "ros::NodeHandle" not in content
    assert "registerInterface" not in content
