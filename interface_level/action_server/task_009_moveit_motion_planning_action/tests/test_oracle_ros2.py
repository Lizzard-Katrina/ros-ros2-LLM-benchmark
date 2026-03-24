# test_oracle_task_009.py
import re
import pytest

from pathlib import Path

# -------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------

TRANSLATED_FILE = "move_action_capability.cpp"

def load_code():
    """
    Load the translated ROS2 C++ code produced by the LLM.
    This is the target of the oracle, NOT the original ROS1 code.
    """
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()



@pytest.fixture
def code():
    return load_code()
# ----------------------
# Concept Tests
# ----------------------
def test_class_exists(code):
    """Check if MoveGroupMoveAction class exists"""
    assert re.search(r'class\s+MoveGroupMoveAction', code), \
        "MoveGroupMoveAction class not defined."

def test_action_server_usage(code):
    """Check if rclcpp_action server is created"""
    patterns = [
        r'rclcpp_action::Server<\s*moveit_msgs::action::MoveGroup\s*>\s*::',
        r'create_server<\s*moveit_msgs::action::MoveGroup\s*>'
    ]
    assert any(re.search(p, code) for p in patterns), \
        "ROS2 action server not instantiated with MoveGroup action."

def test_rclcpp_node_exists(code):
    """Check if ROS2 node is created somewhere in the code"""
    node_patterns = [
        r'rclcpp::Node\s*::\w+',
        r'std::make_shared<rclcpp::Node>'
    ]
    assert any(re.search(p, code) for p in node_patterns), \
        "ROS2 node not created."



def test_initialize_creates_server_with_callback(code):
    """Check initialize() exists and server callback is set"""
    assert re.search(r'void\s+MoveGroupMoveAction::initialize\s*\(', code), \
        "initialize() method missing."
    server_patterns = [
        r'rclcpp_action::create_server',
        r'rclcpp_action::Server<.*>::'
    ]
    assert any(re.search(p, code) for p in server_patterns), \
        "initialize() does not create ROS2 action server."
    # Check that callback points to executeMoveCallback
    callback_patterns = [
        r'create_server<.*>\(\s*.*,\s*\[this\].*executeMoveCallback',
        r'Server<.*>::create\(\s*.*,\s*\[this\].*executeMoveCallback'
    ]
    assert any(re.search(p, code) for p in callback_patterns), \
        "Server callback does not point to executeMoveCallback."

def test_execute_callback_sets_result(code):
    """Check executeMoveCallback sets goal result and handles plan_only / plan_and_execute"""
    assert re.search(r'void\s+MoveGroupMoveAction::executeMoveCallback\s*\(', code), \
        "executeMoveCallback() method missing."
    # Check at least one result is set
    result_patterns = [
        r'goal_handle->succeed\(',
        r'goal_handle->abort\(',
        r'goal_handle->canceled\('
    ]
    assert any(re.search(p, code) for p in result_patterns), \
        "executeMoveCallback() does not set action result (succeed/abort/cancel)."
    # Optional: plan_only or plan_and_execute logic
    plan_patterns = [
        r'if\s*\(.*plan_only.*\)',
        r'planAndExecute'
    ]
    assert any(re.search(p, code) for p in plan_patterns), \
        "executeMoveCallback() missing plan_only / plan_and_execute handling."

def test_preempt_callback_handles_cancel_and_flag(code):
    """Check preemptMoveCallback handles cancel and sets preempt_requested"""
    assert re.search(r'void\s+MoveGroupMoveAction::preemptMoveCallback\s*\(', code), \
        "preemptMoveCallback() method missing."
    preempt_patterns = [
        r'preempt_requested\s*=\s*true',
        r'goal_handle->canceled\(',
        r'plan_execution->stop\('
    ]
    assert any(re.search(p, code) for p in preempt_patterns), \
        "preemptMoveCallback() does not handle preemption/cancel properly."

def test_setMoveState_publishes_feedback_with_state(code):
    """Check setMoveState publishes feedback with correct state"""
    assert re.search(r'void\s+MoveGroupMoveAction::setMoveState\s*\(', code), \
        "setMoveState() method missing."
    feedback_patterns = [
        r'goal_handle->publish_feedback\(',
        r'rclcpp_action::GoalHandle<.*>::publish_feedback\(',
        r'move_feedback\.state\s*='
    ]
    assert any(re.search(p, code) for p in feedback_patterns), \
        "setMoveState() does not publish feedback or set state."

def test_no_ros1_artifacts(code):
    """Ensure no ROS1 remnants"""
    ros1_patterns = [
        r'#include\s*<ros/ros\.h>',
        r'ROS_INFO',
        r'boost::shared_ptr',
        r'ros::init',
        r'ros::NodeHandle'
    ]
    for p in ros1_patterns:
        assert not re.search(p, code), f"Found ROS1 artifact: {p}"
