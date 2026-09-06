"""
Runtime test for the pick_and_place_server ROS2 translation.
Tests that the module can be imported, the node can be instantiated,
and the key methods and structures exist with correct behavior.
"""

import pytest
import re
import sys
import os
import time
import inspect
from pathlib import Path


def test_module_import():
    """Test that the translated module can be imported."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import (
        PickAndPlaceServer,
        createPickupGoal,
        createPlaceGoal,
        moveit_error_dict,
    )
    assert PickAndPlaceServer is not None
    assert callable(createPickupGoal)
    assert callable(createPlaceGoal)
    assert isinstance(moveit_error_dict, dict)


def test_no_rospy_imports():
    """Verify no ROS1 imports remain."""
    from task_006_3d_sensor_moveit_arm_control import pick_and_place_server
    source_path = Path(pick_and_place_server.__file__)
    source = source_path.read_text()
    assert "rospy" not in source.lower(), "ROS1 rospy references found in source"
    assert "import rclpy" in source, "Missing rclpy import"


def test_node_class_inherits_from_node():
    """Verify PickAndPlaceServer inherits from rclpy Node."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import PickAndPlaceServer
    from rclpy.node import Node
    assert issubclass(PickAndPlaceServer, Node), "PickAndPlaceServer must inherit from rclpy.node.Node"


def test_create_pickup_goal_structure():
    """Test createPickupGoal returns a proper goal."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import createPickupGoal
    from task_006_3d_sensor_moveit_arm_control._moveit_compat import Grasp
    from geometry_msgs.msg import PoseStamped

    grasp = Grasp()
    grasp.id = "test_grasp"
    ps = PoseStamped()

    goal = createPickupGoal(
        group="arm_torso",
        target="test_part",
        grasp_pose=ps,
        possible_grasps=[grasp],
        links_to_allow_contact=["link1", "link2"]
    )

    assert goal.target_name == "test_part"
    assert goal.group_name == "arm_torso"
    assert len(goal.possible_grasps) == 1
    assert goal.allowed_planning_time == 35.0
    assert goal.planning_options.replan is True
    assert '<octomap>' in goal.attached_object_touch_links
    assert 'link1' in goal.attached_object_touch_links


def test_create_place_goal_structure():
    """Test createPlaceGoal returns a proper goal."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import createPlaceGoal
    from task_006_3d_sensor_moveit_arm_control._moveit_compat import PlaceLocation
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    pl = PlaceLocation()
    pl.place_pose = ps

    goal = createPlaceGoal(
        place_pose=ps,
        place_locations=[pl],
        group="arm_torso",
        target="test_part",
        links_to_allow_contact=["link1"]
    )

    assert goal.group_name == "arm_torso"
    assert goal.attached_object_name == "test_part"
    assert len(goal.place_locations) == 1
    assert goal.allowed_planning_time == 15.0
    assert '<octomap>' in goal.allowed_touch_objects


def test_wait_for_planning_scene_object_is_async():
    """Verify wait_for_planning_scene_object is an async method with correct logic."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import PickAndPlaceServer
    method = getattr(PickAndPlaceServer, 'wait_for_planning_scene_object')
    assert inspect.iscoroutinefunction(method), "wait_for_planning_scene_object must be async"


def test_grasp_object_is_async():
    """Verify grasp_object is an async method."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import PickAndPlaceServer
    method = getattr(PickAndPlaceServer, 'grasp_object')
    assert inspect.iscoroutinefunction(method), "grasp_object must be async"


def test_place_object_is_async():
    """Verify place_object is an async method."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import PickAndPlaceServer
    method = getattr(PickAndPlaceServer, 'place_object')
    assert inspect.iscoroutinefunction(method), "place_object must be async"


def test_source_oracle_checks():
    """Run the same checks as the oracle test to ensure they pass."""
    from task_006_3d_sensor_moveit_arm_control import pick_and_place_server
    source_path = Path(pick_and_place_server.__file__)
    source = source_path.read_text()

    # Test 1: Pure ROS2
    assert "rospy" not in source.lower()
    assert "import rclpy" in source
    assert re.search(r"class\s+\w+\s*\(.*?Node.*?\):", source)

    # Test 2: Wait logic
    def get_function_block(func_name):
        pattern = rf"(?:async\s+)?def\s+{func_name}\s*\(.*?\):([\s\S]*?)(?=\n    async\s+def|\n    def|\Z)"
        match = re.search(pattern, source)
        return match.group(1) if match else ""

    blk = get_function_block("wait_for_planning_scene_object")
    assert re.search(r"while\s+", blk), "Missing while loop in wait_for_planning_scene_object"
    assert re.search(r"(call|get_planning_scene)", blk), "Missing service call"
    assert re.search(r"\.id\s*==\s*object_name", blk), "Missing ID comparison"

    # Test 3: Grasp sequence
    blk = get_function_block("grasp_object")
    integrity_pattern = (
        r"remove[\s\S]*?"
        r"add_box[\s\S]*?add_box[\s\S]*?"
        r"wait_for_planning_scene"
    )
    assert re.search(integrity_pattern, blk), "Grasp sequence violated"

    # Test 4: Async fallback
    blk = get_function_block("place_object")
    strict_fallback = (
        r"await[\s\S]*?"
        r"if\s+[\s\S]*?['\"]arm_torso['\"][\s\S]*?"
        r"await[\s\S]*?"
        r"return\s+.*?(?:code|val|result)"
    )
    assert re.search(strict_fallback, blk), "Async fallback structure missing"


def test_moveit_error_dict_populated():
    """Verify moveit_error_dict is populated with known error codes."""
    from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import moveit_error_dict
    # MoveItErrorCodes.SUCCESS = 1
    assert 1 in moveit_error_dict
    assert moveit_error_dict[1] == "SUCCESS"


def test_node_instantiation_and_destruction():
    """Test that the node can be created and destroyed with rclpy."""
    import rclpy
    rclpy.init()
    try:
        from task_006_3d_sensor_moveit_arm_control.pick_and_place_server import PickAndPlaceServer
        node = PickAndPlaceServer()
        assert node.get_name() == 'pick_and_place_server'
        assert node.object_height == 0.1
        assert node.object_width == 0.05
        assert node.object_depth == 0.05
        assert isinstance(node.links_to_allow_contact, list)
        node.destroy_node()
    finally:
        rclpy.shutdown()