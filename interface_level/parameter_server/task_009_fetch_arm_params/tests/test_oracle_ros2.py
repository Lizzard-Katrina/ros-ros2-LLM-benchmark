import re
import pytest
from pathlib import Path

# Assuming the file is named fetch_ik_plugin.cpp
CPP_FILE = Path(__file__).resolve().parents[1] / "fetch_arm_ikfast_moveit_plugin.cpp"

def get_code():
    with open(CPP_FILE, "r") as f:
        return f.read()

def test_ros2_param_declaration():
    """Verify LLM understands ROS 2 'Declare before Get' requirement."""
    code = get_code()
    # Check for declaration of either robot_description or solver settings
    pattern = r"node_->declare_parameter\s*<\s*(?:std::string|double|int)\s*>\s*\(\s*\"[\w_]+\""
    assert re.search(pattern, code), "Missing mandatory ROS 2 parameter declarations via node_->declare_parameter."

def test_snake_case_naming():
    """Verify parameters follow ROS 2 snake_case convention, not ROS 1 CamelCase."""
    code = get_code()
    assert "robot_description" in code, "Parameter 'robot_description' should be snake_case."
    assert "robotDescription" not in code, "Detected legacy ROS 1 CamelCase parameter naming."

def test_logging_migration():
    """Verify transition from ROS_ERROR/INFO to node-based RCLCPP macros."""
    code = get_code()
    # Should find RCLCPP macros but NOT the old ROS_ macros
    assert re.search(r"RCLCPP_(?:ERROR|INFO|DEBUG|WARN)", code), "No ROS 2 logging macros (RCLCPP_*) found."
    assert "ROS_ERROR" not in code, "Legacy ROS 1 logging (ROS_ERROR) should be removed."
    assert "get_logger()" in code, "RCLCPP macros must use node_->get_logger()."

def test_frame_consistency():
    """Verify the solver is linked to its hardcoded generated frames."""
    code = get_code()
    # The solver logic MUST reference the hardcoded frames to calculate offsets
    assert "IKFAST_BASE_FRAME_" in code, "Initialization must account for IKFAST_BASE_FRAME_."
    assert "IKFAST_TIP_FRAME_" in code, "Initialization must account for IKFAST_TIP_FRAME_."

def test_moveit2_api_usage():
    """Verify usage of MoveIt 2 RobotModel/JointModelGroup API."""
    code = get_code()
    # Common MoveIt 2 methods for kinematics initialization
    assert "getJointModelGroup" in code, "Missing call to getJointModelGroup for validation."
    assert "getVariableBounds" in code or "getJointLimits" in code.lower(), "Should retrieve joint limits from RobotModel."

def test_tip_frame_validation():
    """Verify semantic check for IKFast 6DOF constraint (1 tip frame)."""
    code = get_code()
    # IKFast typically supports exactly one tip; LLM should check tip_frames.size()
    assert re.search(r"tip_frames\.size\(\)\s*(!=|==|>)\s*1", code), "Should validate that exactly one tip frame is provided."
