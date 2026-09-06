import os
import re
import pytest
from pathlib import Path

# Path definitions
LAUNCH_PATH = Path(__file__).resolve().parents[1].parent / "robot_interface_simulator.launch"
CPP_STATE_PATH = Path(__file__).resolve().parents[1].parent / "generic_robot_state_node.cpp"
CPP_TRAJ_PATH = Path(__file__).resolve().parents[1].parent / "joint_trajectory_interface.cpp"

def test_ros2_state_node_lifecycle():
    """Validates ROS 2 Smart Pointer instantiation and Executor spin."""
    with open(CPP_STATE_PATH, 'r') as f:
        content = f.read()
    
    # Check for ROS 2 specific headers and namespaces
    assert "rclcpp/rclcpp.hpp" in content
    
    # Validates Smart Pointer usage (std::make_shared) instead of stack allocation
    assert re.search(r"std::make_shared<RobotStateInterface>", content), "Should use std::make_shared for ROS 2 Nodes."
    
    # Validates ROS 2 spinning mechanism
    assert "rclcpp::spin" in content
    
    # CRITICAL: Even in ROS 2, the underlying industrial_core logic often requires 
    # a manual call to an initialization method to bridge with the TCP layer.
    # Check if the node is initialized before spinning.
    assert "->init" in content or ".init" in content, "Missing node->init() call. The TCP connection won't start!"

def test_ros2_trajectory_param_logic():
    """Validates ROS 2 parameter declaration and retrieval."""
    with open(CPP_TRAJ_PATH, 'r') as f:
        content = f.read()
    
    # ROS 2 parameters MUST be declared before use
    assert "declare_parameter" in content, "ROS 2 requires parameters to be declared."
    assert "controller_joint_names" in content
    
    # Check if it uses the modern getJointNames which should be adapted for ROS 2 nodes
    assert "getJointNames" in content

def test_launch_system_migration_check():
    """Validates if the launch file was actually migrated to ROS 2."""
    with open(LAUNCH_PATH, 'r') as f:
        content = f.read()
    
    # If the file starts with <launch>, it's still ROS 1 XML, which is a fail.
    assert "<launch>" not in content, "FAILED: Detected ROS 1 XML format. ROS 2 launch must be Python or YAML."
    
    # For a Python launch file, check for essential ROS 2 launch imports
    if str(LAUNCH_PATH).endswith(".py"):
        assert "launch" in content and "launch_ros" in content, "Missing ROS 2 Launch namespaces."

def test_no_ros1_symbols():
    """Ensures no legacy ROS 1 symbols remained after migration."""
    with open(CPP_STATE_PATH, 'r') as f:
        content = f.read()
    
    legacy_symbols = ["ros::init", "ros::NodeHandle", "ros::spin()", "ros::ok()"]
    for symbol in legacy_symbols:
        assert symbol not in content, f"Legacy ROS 1 symbol '{symbol}' detected in ROS 2 code!"