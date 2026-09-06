"""
Runtime test for task_012_move_arm_cartesian_action.

This test validates the translated ROS2 C++ source file by reading the actual
installed source file and verifying all required ROS2 API patterns are present.

Since this is a MoveIt application that requires a full robot setup (URDF, SRDF,
controllers, etc.) which cannot run in a bare Docker container, we validate
the translation by confirming all required ROS2 API patterns are present in the
REAL translated file. We also verify the package built successfully.
"""

import pytest
import re
import os
import subprocess
from pathlib import Path


def find_source_file():
    """Find the translated move_group_interface.cpp file."""
    # Check relative to this test file (package root)
    test_dir = Path(__file__).parent
    candidates = [
        test_dir / "move_group_interface.cpp",
        test_dir / "src" / "move_group_interface.cpp",
    ]
    for c in candidates:
        if c.exists():
            return c
    # Check installed location
    installed = Path("/ros2_ws/install/task_012_move_arm_cartesian_action/share/task_012_move_arm_cartesian_action/src/move_group_interface.cpp")
    if installed.exists():
        return installed
    # Also search in common colcon workspace locations
    workspace_src = Path("/ros2_ws/src/task_012_move_arm_cartesian_action")
    if (workspace_src / "move_group_interface.cpp").exists():
        return workspace_src / "move_group_interface.cpp"
    # Fallback: search upward
    p = test_dir
    for _ in range(5):
        candidate = p / "move_group_interface.cpp"
        if candidate.exists():
            return candidate
        p = p.parent
    return None


@pytest.fixture(scope="module")
def source_code():
    """Load the translated C++ source code."""
    src = find_source_file()
    assert src is not None, "Could not find move_group_interface.cpp"
    return src.read_text()


def test_file_exists():
    """Verify the translated source file exists."""
    src = find_source_file()
    assert src is not None, "move_group_interface.cpp not found"
    content = src.read_text()
    assert len(content) > 100, "Source file appears to be empty or too short"


def test_package_built():
    """Verify the package built successfully by checking install directory."""
    install_dir = Path("/ros2_ws/install/task_012_move_arm_cartesian_action")
    assert install_dir.exists(), "Package install directory not found - build failed"


def test_ros2_init_present(source_code):
    """Verify ROS2 initialization with rclcpp::init."""
    assert re.search(r"rclcpp::init\s*\(", source_code), \
        "rclcpp::init() call not found"


def test_ros2_node_creation(source_code):
    """Verify a ROS2 node is created."""
    assert re.search(r"rclcpp::Node::make_shared", source_code), \
        "ROS2 node creation not found"


def test_spinning_mechanism(source_code):
    """Verify a spinning mechanism is used."""
    # ROS2 MoveIt tutorials use executor + thread instead of AsyncSpinner
    has_executor = re.search(r"executor", source_code) is not None
    has_spinner = re.search(r"spinner", source_code) is not None
    assert has_executor or has_spinner, \
        "No spinning mechanism found"


def test_move_group_interface_instantiation(source_code):
    """Verify MoveGroupInterface is instantiated with PLANNING_GROUP."""
    pattern = r"moveit::planning_interface::MoveGroupInterface\s+\w+\s*\("
    assert re.search(pattern, source_code), \
        "MoveGroupInterface instantiation not found"


def test_planning_group_defined(source_code):
    """Verify PLANNING_GROUP is defined as panda_arm."""
    assert re.search(r'PLANNING_GROUP\s*=\s*"panda_arm"', source_code), \
        "PLANNING_GROUP definition not found"


def test_pose_target_set(source_code):
    """Verify setPoseTarget is called."""
    assert re.search(r"\w+\.setPoseTarget\s*\(", source_code), \
        "setPoseTarget call not found"


def test_plan_called(source_code):
    """Verify plan() is called."""
    assert re.search(r"\w+\.plan\s*\(", source_code), \
        "plan() call not found"


def test_target_pose1_defined(source_code):
    """Verify target_pose1 is defined with correct values."""
    assert re.search(r"geometry_msgs::msg::Pose\s+target_pose1", source_code), \
        "target_pose1 not defined as geometry_msgs::msg::Pose"
    assert re.search(r"target_pose1\.orientation\.w\s*=\s*1\.0", source_code), \
        "target_pose1 orientation not set"
    assert re.search(r"target_pose1\.position\.x\s*=\s*0\.28", source_code), \
        "target_pose1 position.x not set to 0.28"


def test_joint_value_target(source_code):
    """Verify setJointValueTarget is called."""
    assert re.search(r"\w+\.setJointValueTarget\s*\(", source_code), \
        "setJointValueTarget call not found"


def test_orientation_constraint(source_code):
    """Verify OrientationConstraint is defined."""
    assert re.search(r"moveit_msgs::msg::OrientationConstraint\s+\w+", source_code), \
        "OrientationConstraint not found"


def test_constraints_object(source_code):
    """Verify Constraints object is created."""
    assert re.search(r"moveit_msgs::msg::Constraints\s+\w+", source_code), \
        "Constraints object not found"


def test_set_path_constraints(source_code):
    """Verify setPathConstraints is called."""
    assert re.search(r"\w+\.setPathConstraints\s*\(", source_code), \
        "setPathConstraints call not found"


def test_waypoints_vector(source_code):
    """Verify waypoints vector is defined."""
    assert re.search(r"std::vector<geometry_msgs::msg::Pose>\s+waypoints", source_code), \
        "Waypoints vector not found"


def test_compute_cartesian_path(source_code):
    """Verify computeCartesianPath is called."""
    assert re.search(r"\w+\.computeCartesianPath\s*\(", source_code), \
        "computeCartesianPath call not found"


def test_collision_object_defined(source_code):
    """Verify CollisionObject is defined."""
    assert re.search(r"moveit_msgs::msg::CollisionObject\s+\w+", source_code), \
        "CollisionObject not defined"


def test_add_collision_objects(source_code):
    """Verify addCollisionObjects is called."""
    assert re.search(r"\w+\.addCollisionObjects\s*\(", source_code), \
        "addCollisionObjects call not found"


def test_apply_collision_object(source_code):
    """Verify applyCollisionObject is called."""
    assert re.search(r"\w+\.applyCollisionObject\s*\(", source_code), \
        "applyCollisionObject call not found"


def test_attach_object(source_code):
    """Verify attachObject is called."""
    assert re.search(r"\w+\.attachObject\s*\(", source_code), \
        "attachObject call not found"


def test_detach_object(source_code):
    """Verify detachObject is called."""
    assert re.search(r"\w+\.detachObject\s*\(", source_code), \
        "detachObject call not found"


def test_visual_tools_instantiation(source_code):
    """Verify MoveItVisualTools is instantiated."""
    assert re.search(r"moveit_visual_tools::MoveItVisualTools\s+\w+", source_code), \
        "MoveItVisualTools instantiation not found"


def test_visual_tools_publish(source_code):
    """Verify visual tools publish methods are called."""
    assert re.search(r"\w+\.publishText\s*\(", source_code), \
        "publishText call not found"
    assert re.search(r"\w+\.publishTrajectoryLine\s*\(", source_code), \
        "publishTrajectoryLine call not found"


def test_ros2_msg_namespace(source_code):
    """Verify ROS2 message namespaces use ::msg:: instead of ROS1 style."""
    assert re.search(r"geometry_msgs::msg::", source_code), \
        "geometry_msgs::msg:: namespace not found (still using ROS1 style?)"
    assert re.search(r"moveit_msgs::msg::", source_code), \
        "moveit_msgs::msg:: namespace not found (still using ROS1 style?)"


def test_rclcpp_shutdown(source_code):
    """Verify rclcpp::shutdown is called."""
    assert re.search(r"rclcpp::shutdown\s*\(", source_code), \
        "rclcpp::shutdown() not found"


def test_rclcpp_info_logging(source_code):
    """Verify ROS2 logging macros are used instead of ROS1."""
    assert re.search(r"RCLCPP_INFO\s*\(", source_code), \
        "RCLCPP_INFO logging not found"


def test_no_ros1_artifacts(source_code):
    """Verify no ROS1 artifacts remain."""
    assert not re.search(r"ros::init\s*\(", source_code), \
        "ROS1 ros::init still present"
    assert not re.search(r"ros::NodeHandle", source_code), \
        "ROS1 ros::NodeHandle still present"
    assert not re.search(r"ROS_INFO_NAMED\s*\(", source_code), \
        "ROS1 ROS_INFO_NAMED still present"


def test_cylinder_object_defined(source_code):
    """Verify the cylinder collision object for attachment is defined."""
    assert re.search(r'object_to_attach', source_code), \
        "object_to_attach not found in source"
    assert re.search(r'"cylinder1"', source_code), \
        "cylinder1 id not found"


def test_touch_links_defined(source_code):
    """Verify touch links for gripper fingers are defined."""
    assert re.search(r"panda_rightfinger", source_code), \
        "panda_rightfinger touch link not found"
    assert re.search(r"panda_leftfinger", source_code), \
        "panda_leftfinger touch link not found"


def test_remove_collision_objects(source_code):
    """Verify removeCollisionObjects is called."""
    assert re.search(r"\w+\.removeCollisionObjects\s*\(", source_code), \
        "removeCollisionObjects call not found"


def test_cartesian_waypoints_content(source_code):
    """Verify waypoints include position modifications."""
    assert re.search(r"position\.z\s*-=\s*0\.2", source_code), \
        "Waypoint z -= 0.2 not found"
    assert re.search(r"position\.y\s*-=\s*0\.2", source_code), \
        "Waypoint y -= 0.2 not found"


def test_ros2_includes(source_code):
    """Verify ROS2-style includes are used."""
    assert re.search(r'#include\s*<moveit_msgs/msg/', source_code), \
        "ROS2-style moveit_msgs/msg/ include not found"


def test_executor_based_spinning(source_code):
    """Verify executor-based spinning (ROS2 pattern) instead of ros::AsyncSpinner."""
    # Should not have ros::AsyncSpinner
    assert not re.search(r"ros::AsyncSpinner", source_code), \
        "ROS1 ros::AsyncSpinner still present"
    # Should have some form of executor or thread-based spinning
    has_executor = re.search(r"executor", source_code, re.IGNORECASE) is not None
    has_spin = re.search(r"\.spin\(\)", source_code) is not None
    assert has_executor or has_spin, \
        "No executor-based spinning mechanism found"