import re
import pytest


from pathlib import Path

# -------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------

TRANSLATED_FILE = "move_group_interface.cpp"

def load_code():
    """
    Load the translated ROS2 C++ code produced by the LLM.
    This is the target of the oracle, NOT the original ROS1 code.
    """
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()

code=load_code()

def test_ros2_node_initialization():
    """Check that a ROS2 node is initialized and spinning asynchronously"""
    pattern = r"rclcpp::init\s*\(.*\).*?rclcpp::AsyncSpinner"
    assert re.search(pattern, code, re.DOTALL), "ROS2 node or async spinner missing"

def test_move_group_interface_exists():
    """Check that MoveGroupInterface for a planning group is created"""
    pattern = r"moveit::planning_interface::MoveGroupInterface\s+\w+\s*\(\s*PLANNING_GROUP\s*\)"
    assert re.search(pattern, code), "MoveGroupInterface instance missing"

def test_pose_target_planning():
    """Check that a Pose target is set and a plan is generated"""
    pattern_set_pose = r"\w+\.setPoseTarget\s*\("
    pattern_plan = r"\w+\.plan\s*\("
    assert re.search(pattern_set_pose, code), "Pose target not set"
    assert re.search(pattern_plan, code), "Planning function call missing for pose target"

def test_joint_space_planning():
    """Check that a joint space goal is set and planned"""
    pattern_joint_target = r"\w+\.setJointValueTarget\s*\("
    pattern_plan = r"\w+\.plan\s*\("
    assert re.search(pattern_joint_target, code), "Joint space target not set"
    assert re.search(pattern_plan, code), "Planning function call missing for joint space target"

def test_path_constraints():
    """Check that orientation constraints are defined and applied"""
    pattern_constraints = r"moveit_msgs::Constraints\s+\w+"
    pattern_ocm = r"moveit_msgs::OrientationConstraint\s+\w+"
    assert re.search(pattern_constraints, code), "Path constraints object missing"
    assert re.search(pattern_ocm, code), "OrientationConstraint missing"

def test_cartesian_path_computation():
    """Check that a Cartesian path is computed from waypoints"""
    pattern_vector = r"std::vector<geometry_msgs::Pose>\s+\w+"
    pattern_compute = r"\w+\.computeCartesianPath\s*\("
    assert re.search(pattern_vector, code), "Waypoints vector missing"
    assert re.search(pattern_compute, code), "Cartesian path computation missing"

def test_collision_object_added():
    """Check that a collision object is defined and added to the planning scene"""
    pattern_object = r"moveit_msgs::CollisionObject\s+\w+"
    pattern_add = r"\w+\.addCollisionObjects\s*\("
    pattern_apply = r"\w+\.applyCollisionObject\s*\("
    assert re.search(pattern_object, code), "Collision object not defined"
    assert re.search(pattern_add, code), "Collision object not added to planning scene"
    assert re.search(pattern_apply, code), "Collision object not applied with applyCollisionObject()"

def test_object_attached_to_robot():
    """Check that an object is attached to the robot gripper"""
    pattern_attach = r"\w+\.attachObject\s*\("
    pattern_detach = r"\w+\.detachObject\s*\("
    assert re.search(pattern_attach, code), "Object attachment missing"
    assert re.search(pattern_detach, code), "Object detachment missing"

def test_visual_tools_usage():
    """Check that MoveItVisualTools is instantiated and used to publish markers"""
    pattern_vt = r"moveit_visual_tools::MoveItVisualTools\s+\w+"
    pattern_publish = r"\w+\.publish.*\("
    assert re.search(pattern_vt, code), "MoveItVisualTools instance missing"
    assert re.search(pattern_publish, code), "Visualization publish calls missing"
