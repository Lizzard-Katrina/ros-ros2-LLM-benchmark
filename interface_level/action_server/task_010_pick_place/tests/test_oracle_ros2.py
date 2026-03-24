# src/task_013/test/test_oracle_ros2_action_server.py
import re
import pytest

from pathlib import Path

# -------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------

TRANSLATED_FILE = "pick_place.cpp"

def load_code():
    """
    Load the translated ROS2 C++ code produced by the LLM.
    This is the target of the oracle, NOT the original ROS1 code.
    """
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()

code=load_code()

def test_move_group_interface_created():
    """Check MoveGroupInterface for panda_arm is created"""
    pattern = r"moveit::planning_interface::MoveGroupInterface\s+\w+\s*\(\s*\"panda_arm\"\s*\)"
    assert re.search(pattern, code), "MoveGroupInterface instance missing"

def test_ros2_node_and_spinner():
    """Check that a ROS2 node is initialized and spinning asynchronously"""
    pattern = r"rclcpp::init\s*\(.*\).*?rclcpp::AsyncSpinner"
    assert re.search(pattern, code, re.DOTALL), "ROS2 node or async spinner missing"

def test_pick_place_sequence():
    """Check that pick() is called before place() on the correct MoveGroupInterface instance"""
    # pick and place calls must exist
    pick_pattern = r"\w+\.pick\s*\("
    place_pattern = r"\w+\.place\s*\("
    pick_match = re.search(pick_pattern, code)
    place_match = re.search(place_pattern, code)
    assert pick_match, "pick() function missing"
    assert place_match, "place() function missing"
    # pick must appear before place
    assert pick_match.start() < place_match.start(), "pick() should be called before place()"

def test_gripper_operations():
    """Check that openGripper and closedGripper are called in correct phases"""
    pre_grasp_pattern = r"openGripper\s*\(\s*\w+\.pre_grasp_posture\s*\)"
    grasp_pattern = r"closedGripper\s*\(\s*\w+\.grasp_posture\s*\)"
    post_place_pattern = r"openGripper\s*\(\s*\w+\.post_place_posture\s*\)"
    assert re.search(pre_grasp_pattern, code), "openGripper for pre_grasp_posture missing"
    assert re.search(grasp_pattern, code), "closedGripper for grasp_posture missing"
    assert re.search(post_place_pattern, code), "openGripper for post_place_posture missing"

def test_collision_objects_added():
    """Check that table1, table2, and object collision objects are added"""
    table1_pattern = r'collision_objects\[0\]\.id\s*=\s*"table1"'
    table2_pattern = r'collision_objects\[1\]\.id\s*=\s*"table2"'
    object_pattern = r'collision_objects\[2\]\.id\s*=\s*"object"'
    apply_pattern = r'planning_scene_interface\.applyCollisionObjects\s*\('
    assert re.search(table1_pattern, code), "table1 collision object missing"
    assert re.search(table2_pattern, code), "table2 collision object missing"
    assert re.search(object_pattern, code), "manipulated object missing"
    assert re.search(apply_pattern, code), "Collision objects not applied to planning scene"

def test_post_place_open_gripper():
    """Ensure openGripper is called after place() for post_place_posture"""
    place_pattern = r"\w+\.place\s*\("
    post_place_pattern = r"openGripper\s*\(\s*\w+\.post_place_posture\s*\)"
    place_match = re.search(place_pattern, code)
    post_place_match = re.search(post_place_pattern, code)
    assert post_place_match, "openGripper for post_place_posture missing"
    assert place_match.start() < post_place_match.start(), "post_place openGripper should be after place()"
