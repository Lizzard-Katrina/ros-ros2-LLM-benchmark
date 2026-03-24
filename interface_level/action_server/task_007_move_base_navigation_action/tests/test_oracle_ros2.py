import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1]/"move_base.cpp"

def read_cpp_file():
    with open(CPP_FILE, "r") as f:
        return f.read()

# ----------------------------
# Oracle tests for ROS2 move_base translation
# ----------------------------

def test_mutex_translation():
    """Check that boost::recursive_mutex::scoped_lock is updated to ROS2-friendly locks"""
    code = read_cpp_file()
    # In ROS2, we typically use std::mutex/std::unique_lock or rclcpp::Mutex
    pattern = r'(std::unique_lock|rclcpp::Mutex|std::lock_guard)'
    assert re.search(pattern, code), "ROS2 translation should replace recursive_mutex locks with std::unique_lock or ROS2 equivalents."

def test_feedback_publishing_exists():
    """Check that move_base_msgs::MoveBaseFeedback publishing still exists"""
    code = read_cpp_file()
    pattern = r'as_->publishFeedback'
    assert re.search(pattern, code), "ROS2 code must still publish feedback via action server."

def test_zero_velocity_publish_function():
    """Check that publishZeroVelocity function is used"""
    code = read_cpp_file()
    pattern = r'publishZeroVelocity\s*\(\s*\)'
    assert re.search(pattern, code), "ROS2 code must still use publishZeroVelocity for safety stops."

def test_plan_swap_under_lock():
    """Check that plan pointer swap occurs under a mutex"""
    code = read_cpp_file()
    patterns = [
        r'controller_plan_ = latest_plan_',
        r'latest_plan_ = temp_plan',
        r'(std::unique_lock|rclcpp::Mutex)'
    ]
    for pat in patterns:
        assert re.search(pat, code), f"Plan swap under mutex missing: {pat}"

def test_velocity_command_computation():
    """Check that computeVelocityCommands is still called"""
    code = read_cpp_file()
    pattern = r'computeVelocityCommands\s*\('
    assert re.search(pattern, code), "ROS2 code must still call computeVelocityCommands."

def test_state_machine_structure():
    """Check that the PLANNING / CONTROLLING / CLEARING switch still exists"""
    code = read_cpp_file()
    states = ['PLANNING', 'CONTROLLING', 'CLEARING']
    for state in states:
        assert re.search(rf'\b{state}\b', code), f"ROS2 code missing state: {state}"

def test_recovery_behavior_loop():
    """Check that recovery behaviors are iterated correctly"""
    code = read_cpp_file()
    pattern = r'recovery_behaviors_\[recovery_index_\]->runBehavior'
    assert re.search(pattern, code), "ROS2 code must still iterate over recovery behaviors and call runBehavior()."

def test_ros2_logging():
    """Check that ROS1 logging macros replaced with ROS2"""
    code = read_cpp_file()
    forbidden_ros1 = [r'ROS_DEBUG', r'ROS_WARN', r'ROS_ERROR']
    for pat in forbidden_ros1:
        assert not re.search(pat, code), f"ROS1 logging macro found: {pat}"

    # Check at least one rclcpp logging usage
    pattern = r'rclcpp::Logger|RCLCPP_'
    assert re.search(pattern, code), "ROS2 logging (RCLCPP_*) must be used."

def test_action_server_success_abort_calls():
    """Check that as_->setSucceeded and as_->setAborted are present"""
    code = read_cpp_file()
    patterns = ['as_->setSucceeded', 'as_->setAborted']
    for pat in patterns:
        assert re.search(pat, code), f"Action server success/abort call missing: {pat}"
