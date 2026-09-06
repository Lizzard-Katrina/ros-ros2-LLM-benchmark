"""
Runtime test for task_010_pick_place.

This C++ MoveIt package requires a full robot setup (URDF, controllers,
move_group action server, etc.) to actually execute. We verify the translation
by reading the INSTALLED source file and validating its content matches
the expected ROS2 pick-place pipeline semantics. We also verify the package
built and installed successfully.
"""

import re
import os
import subprocess
import glob
import pytest
from pathlib import Path


def _find_pick_place_cpp():
    """Find the translated pick_place.cpp file from the installed share directory or source."""
    candidates = []

    # Check installed share location
    share_paths = glob.glob("/ros2_ws/install/task_010_pick_place/share/task_010_pick_place/src/pick_place.cpp")
    candidates.extend(share_paths)

    # Check source locations
    source_candidates = [
        Path(__file__).parent / "src" / "pick_place.cpp",
        Path(__file__).parent / "pick_place.cpp",
    ]
    for c in source_candidates:
        if c.exists():
            candidates.append(str(c))

    # Also search broadly
    result = subprocess.run(
        ["find", "/ros2_ws", "-name", "pick_place.cpp", "-path", "*task_010*"],
        capture_output=True, text=True, timeout=10
    )
    for p in result.stdout.strip().split('\n'):
        p = p.strip()
        if p and os.path.isfile(p):
            candidates.append(p)

    for c in candidates:
        path = Path(c)
        if path.exists():
            content = path.read_text()
            if "rclcpp" in content:
                return content

    pytest.fail("Could not find pick_place.cpp with ROS2 content in package")


@pytest.fixture(scope="module")
def code():
    return _find_pick_place_cpp()


class TestPickPlacePackageInstalled:
    """Verify the package was built and installed."""

    def test_package_installed(self):
        """Check that the package directory exists in the install space."""
        install_dir = Path("/ros2_ws/install/task_010_pick_place")
        assert install_dir.exists(), "Package not installed"

    def test_source_installed(self):
        """Check that the source file was installed to share."""
        installed = Path("/ros2_ws/install/task_010_pick_place/share/task_010_pick_place/src/pick_place.cpp")
        assert installed.exists(), "pick_place.cpp not installed to share directory"


class TestPickPlaceTranslation:
    """Verify the translated C++ source has correct ROS2 semantics."""

    def test_rclcpp_init_present(self, code):
        assert re.search(r"rclcpp::init\s*\(", code), "rclcpp::init() not found"

    def test_rclcpp_shutdown_present(self, code):
        assert re.search(r"rclcpp::shutdown\s*\(", code), "rclcpp::shutdown() not found"

    def test_node_creation(self, code):
        assert re.search(r'rclcpp::Node::make_shared\s*\(\s*"panda_arm_pick_place"\s*\)', code), \
            "Node creation not found"

    def test_move_group_interface_panda_arm(self, code):
        pattern = r'MoveGroupInterface\s+\w+\s*\([^)]*"panda_arm"'
        assert re.search(pattern, code), "MoveGroupInterface for panda_arm not found"

    def test_planning_scene_interface(self, code):
        assert re.search(r'PlanningSceneInterface', code), "PlanningSceneInterface not found"

    def test_open_gripper_function(self, code):
        assert re.search(r'void\s+openGripper\s*\(', code), "openGripper function not found"

    def test_closed_gripper_function(self, code):
        assert re.search(r'void\s+closedGripper\s*\(', code), "closedGripper function not found"

    def test_open_gripper_pre_grasp(self, code):
        pattern = r"openGripper\s*\(\s*\w+\[0\]\.pre_grasp_posture\s*\)"
        assert re.search(pattern, code), "openGripper(pre_grasp_posture) not found"

    def test_closed_gripper_grasp(self, code):
        pattern = r"closedGripper\s*\(\s*\w+\[0\]\.grasp_posture\s*\)"
        assert re.search(pattern, code), "closedGripper(grasp_posture) not found"

    def test_open_gripper_post_place(self, code):
        pattern = r"openGripper\s*\(\s*\w+\[0\]\.post_place_posture\s*\)"
        assert re.search(pattern, code), "openGripper(post_place_posture) not found"

    def test_pick_function(self, code):
        assert re.search(r'void\s+pick\s*\(', code), "pick function not found"

    def test_place_function(self, code):
        assert re.search(r'void\s+place\s*\(', code), "place function not found"

    def test_pick_called(self, code):
        pattern = r'\w+\.pick\s*\(\s*"object"'
        assert re.search(pattern, code), "pick(\"object\") call not found"

    def test_place_called(self, code):
        pattern = r'\w+\.place\s*\(\s*"object"'
        assert re.search(pattern, code), "place(\"object\") call not found"

    def test_collision_objects_table1(self, code):
        assert re.search(r'"table1"', code), "table1 missing"

    def test_collision_objects_table2(self, code):
        assert re.search(r'"table2"', code), "table2 missing"

    def test_collision_objects_object(self, code):
        assert re.search(r'collision_objects\[2\]\.id\s*=\s*"object"', code), "object missing"

    def test_apply_collision_objects(self, code):
        assert re.search(r'applyCollisionObjects\s*\(', code), "applyCollisionObjects not called"

    def test_ros2_message_types(self, code):
        assert re.search(r"trajectory_msgs::msg::JointTrajectory", code), \
            "ROS2 trajectory_msgs::msg::JointTrajectory not found"

    def test_moveit_msgs_ros2(self, code):
        assert re.search(r"moveit_msgs::msg::Grasp", code), \
            "ROS2 moveit_msgs::msg::Grasp not found"

    def test_moveit_msgs_place_location(self, code):
        assert re.search(r"moveit_msgs::msg::PlaceLocation", code), \
            "ROS2 moveit_msgs::msg::PlaceLocation not found"

    def test_moveit_msgs_collision_object(self, code):
        assert re.search(r"moveit_msgs::msg::CollisionObject", code), \
            "ROS2 moveit_msgs::msg::CollisionObject not found"

    def test_rclcpp_duration(self, code):
        assert not re.search(r"ros::Duration", code), "ROS1 ros::Duration still present"
        assert re.search(r"rclcpp::Duration", code), "rclcpp::Duration not found"

    def test_no_ros1_includes(self, code):
        assert not re.search(r"#include\s*<ros/ros\.h>", code), "ROS1 header still present"

    def test_rclcpp_include(self, code):
        assert re.search(r"#include\s*<rclcpp/rclcpp\.hpp>", code), "rclcpp header missing"

    def test_no_ros1_init(self, code):
        assert not re.search(r"ros::init\s*\(", code), "ROS1 ros::init still present"

    def test_no_ros1_nodehandle(self, code):
        assert not re.search(r"ros::NodeHandle", code), "ROS1 ros::NodeHandle still present"

    def test_tf2_geometry_msgs_hpp(self, code):
        assert re.search(r"tf2_geometry_msgs\.hpp", code), "tf2_geometry_msgs.hpp include missing"

    def test_set_planning_time(self, code):
        assert re.search(r"setPlanningTime\s*\(\s*45\.0\s*\)", code), "setPlanningTime(45.0) not found"

    def test_support_surface_table1(self, code):
        assert re.search(r'setSupportSurfaceName\s*\(\s*"table1"\s*\)', code), \
            "setSupportSurfaceName(table1) not found"

    def test_support_surface_table2(self, code):
        assert re.search(r'setSupportSurfaceName\s*\(\s*"table2"\s*\)', code), \
            "setSupportSurfaceName(table2) not found"

    def test_panda_finger_joints(self, code):
        assert re.search(r'"panda_finger_joint1"', code), "panda_finger_joint1 not found"
        assert re.search(r'"panda_finger_joint2"', code), "panda_finger_joint2 not found"

    def test_frame_id_panda_link0(self, code):
        assert re.search(r'"panda_link0"', code), "panda_link0 frame_id not found"

    def test_sleep_for_ros2(self, code):
        assert re.search(r"rclcpp::sleep_for", code), "rclcpp::sleep_for not found"

    def test_no_ros1_sleep(self, code):
        assert not re.search(r"ros::WallDuration", code), "ROS1 ros::WallDuration still present"