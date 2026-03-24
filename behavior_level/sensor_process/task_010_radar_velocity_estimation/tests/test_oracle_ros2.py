import re
import pytest
from pathlib import Path

# Path to the generated code
CPP_FILE = Path(__file__).resolve().parents[1] / "radar_ego_velocity_estimator_ros.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()

def test_ros2_logging_migration():
    """Verify migration to RCLCPP macros with explicit node logger context."""
    content = get_content()
    pattern = r"RCLCPP_(?:WARN|ERROR|INFO)(?:_THROTTLE)?\s*\(\s*(?:this->)?get_logger\(\)"
    assert re.search(pattern, content), "Failure: Did not use node-based RCLCPP logging macros."

def test_shared_ptr_dereferencing():
    """Verify correct handling of the specific variable name 'radar_scan_msg'."""
    content = get_content()
    # Enforces the use of the requested variable name and dereferencing
    pattern = r"processRadarData\s*\(\s*\*?radar_scan_msg"
    assert re.search(pattern, content), "Failure: Did not use 'radar_scan_msg' or failed to call processRadarData."

def test_ros2_clock_usage():
    """Verify usage of Node Clock API."""
    content = get_content()
    pattern = r"this->(?:get_clock\(\)->)?now\(\)"
    assert re.search(pattern, content), "Failure: Should use node clock (this->now())."
    assert "ros::Time::now()" not in content, "Failure: Found deprecated ros::Time::now()."

def test_mutex_lock_guard():
    """Verify scoped mutex locking."""
    content = get_content()
    pattern = r"std::lock_guard\s*<std::mutex>\s*\w+\s*\(mutex_\)"
    assert re.search(pattern, content), "Failure: Should use std::lock_guard with mutex_."

def test_trigger_reset_logic_strict():
    """
    Critical Logic Check: Verify trigger_stamp is RESET to an initial state.
    Matches assignment to a null rclcpp::Time or similar to prevent stale data.
    """
    content = get_content()
    # This ensures trigger_stamp is actually cleared, not just read.
    reset_pattern = r"trigger_stamp\s*=\s*(?:rclcpp::Time\(\)|this->get_clock\(\)->now\(\).+?\.min\(\)|rcl_interfaces::msg::Log::INFO)" 
    # Broadening slightly to capture various null-assignments in ROS2
    assert "trigger_stamp =" in content, "Failure: Assignment to trigger_stamp missing."
    assert re.search(r"trigger_stamp\s*=\s*(?:rclcpp::Time|.+min)", content), "Logic failure: trigger_stamp was not reset to null/min."

def test_zero_timestamp_handling():
    """Verify the specific '.sec == 0' syntax requested in TODO."""
    content = get_content()
    pattern = r"\.sec\s*==\s*0"
    assert re.search(pattern, content), "Failure: Did not use the required '.sec == 0' syntax for timestamp checking."

def test_branch_preservation():
    """Verify that the 'run_without_trigger' logic branch was not deleted."""
    content = get_content()
    assert "run_without_trigger" in content, "Logic failure: The run_without_trigger conditional branch was omitted."

def test_no_ros1_namespaces():
    """Ensure no ROS 1 legacy namespaces remain."""
    content = get_content()
    bad_patterns = [r"ros::NodeHandle", r"ros::Subscriber", r"sensor_msgs::PointCloud2ConstPtr"]
    for pattern in bad_patterns:
        assert not re.search(pattern, content), f"Failure: Found ROS 1 legacy code: {pattern}"
