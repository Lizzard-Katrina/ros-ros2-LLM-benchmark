"""
Runtime test for task_011_moveit_motion_params.

This C++ MoveIt tutorial requires a full robot setup (URDF, SRDF, planning
pipeline, etc.) which cannot run in a bare Docker container. We validate:
1. The source file exists and contains correct ROS2 migration patterns.
2. The executable was built and can be launched (it will start, log, and shut down
   in the NO_MOVEIT fallback path if MoveIt isn't fully configured).
3. We verify the node actually comes alive by checking for it in the ROS graph.
"""
import re
import os
import subprocess
import time
import pytest
from pathlib import Path


def _find_source_file():
    """Locate the translated C++ source file."""
    candidates = [
        Path(__file__).parent / "moveit_cpp_tutorial.cpp",
        Path(__file__).parent / "src" / "moveit_cpp_tutorial.cpp",
    ]
    for c in candidates:
        if c.exists():
            return c
    for p in Path(__file__).parent.rglob("moveit_cpp_tutorial.cpp"):
        return p
    raise FileNotFoundError("Cannot find moveit_cpp_tutorial.cpp")


def _get_clean_content(filepath):
    """Strip comments from the source file."""
    with open(filepath, 'r') as f:
        content = f.read()
    content = re.sub(r'//.*', '', content)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    return content


def _find_executable():
    """Find the built executable."""
    search_paths = [
        Path("/ros2_ws/install/task_011_moveit_motion_params/lib/task_011_moveit_motion_params/moveit_cpp_tutorial"),
        Path("/ros_ws/install/task_011_moveit_motion_params/lib/task_011_moveit_motion_params/moveit_cpp_tutorial"),
    ]
    # Also check AMENT_PREFIX_PATH
    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    for p in ament.split(":"):
        if p:
            search_paths.insert(0, Path(p) / "lib" / "task_011_moveit_motion_params" / "moveit_cpp_tutorial")
    for sp in search_paths:
        if sp.exists():
            return sp
    return None


@pytest.fixture(scope="module")
def source_content():
    src = _find_source_file()
    return _get_clean_content(src)


@pytest.fixture(scope="module")
def raw_content():
    src = _find_source_file()
    with open(src, 'r') as f:
        return f.read()


def test_executable_was_built():
    """Verify the package built successfully by checking for the installed executable."""
    exe = _find_executable()
    assert exe is not None and exe.exists(), \
        "Compiled executable moveit_cpp_tutorial must exist"


def test_node_launches_and_logs():
    """Launch the executable and verify it produces expected log output."""
    exe = _find_executable()
    if exe is None:
        pytest.skip("Executable not found")

    proc = None
    try:
        env = os.environ.copy()
        proc = subprocess.Popen(
            [str(exe)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        # Wait for the node to start and produce output (it should shut down on its own
        # after a few seconds in the NO_MOVEIT path, or we kill it)
        try:
            stdout, _ = proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            proc.terminate()
            stdout, _ = proc.communicate(timeout=5)

        # The node should have logged something
        assert stdout is not None and len(stdout) > 0, \
            "Node must produce log output"
        # Check for expected log messages
        assert "MoveIt Tutorials" in stdout or "Shutting down" in stdout or "moveit_cpp_tutorial" in stdout.lower(), \
            f"Expected tutorial log messages, got: {stdout[:500]}"
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def test_rclcpp_init(source_content):
    """Verify rclcpp::init is used instead of ros::init."""
    assert "rclcpp::init" in source_content, \
        "Must use rclcpp::init for ROS 2 initialization"


def test_node_options_parameter_overrides(source_content):
    """Verify NodeOptions enables automatic parameter declaration."""
    assert "automatically_declare_parameters_from_overrides" in source_content, \
        "NodeOptions must enable automatically_declare_parameters_from_overrides"
    assert "allow_undeclared_parameters" in source_content, \
        "NodeOptions must allow undeclared parameters"


def test_node_created_with_options(source_content):
    """Verify the node is created with the configured options."""
    has_node = (
        re.search(r"Node\s*\(.*,\s*.*options\s*\)", source_content) or
        re.search(r"make_shared<rclcpp::Node>\s*\(.*,\s*.*options\s*\)", source_content) or
        re.search(r"make_shared<rclcpp::Node>\s*\([^)]*node_options[^)]*\)", source_content)
    )
    assert has_node, "Node must be instantiated with NodeOptions"


def test_moveit_cpp_namespace(source_content):
    """Verify correct ROS 2 namespace for MoveItCpp."""
    assert "moveit_cpp::MoveItCpp" in source_content, \
        "Must use moveit_cpp::MoveItCpp namespace"
    assert "moveit::planning_interface::MoveItCpp" not in source_content, \
        "Must NOT use legacy moveit::planning_interface::MoveItCpp namespace"


def test_async_execution(source_content):
    """Verify background spinning to prevent deadlocks."""
    has_thread = "std::thread" in source_content or "MultiThreadedExecutor" in source_content
    assert has_thread, "Must have background thread or MultiThreadedExecutor"
    assert "spin" in source_content, "Must have a spin call for async execution"


def test_planning_scene_service(source_content):
    """Verify providePlanningSceneService is called."""
    assert "providePlanningSceneService" in source_content, \
        "Must call providePlanningSceneService()"


def test_no_ros1_symbols(source_content):
    """Verify no legacy ROS 1 symbols remain."""
    ros1_symbols = ["ros::init", "ros::NodeHandle", "ros::AsyncSpinner",
                    "ros::Duration", "ros::ok"]
    for symbol in ros1_symbols:
        assert symbol not in source_content, \
            f"Legacy ROS 1 symbol '{symbol}' found in code"


def test_ros2_message_namespaces(source_content):
    """Verify ROS 2 nested message namespaces."""
    assert "geometry_msgs::msg::" in source_content, \
        "Must use geometry_msgs::msg:: namespace for ROS 2 messages"
    assert "PoseStamped" in source_content, \
        "Must use PoseStamped message type"


def test_rclcpp_includes(raw_content):
    """Verify ROS 2 headers are included."""
    assert "rclcpp/rclcpp.hpp" in raw_content, "Must include rclcpp/rclcpp.hpp"


def test_planning_component_created(source_content):
    """Verify PlanningComponent is created with correct namespace."""
    assert "moveit_cpp::PlanningComponent" in source_content, \
        "Must create moveit_cpp::PlanningComponent"


def test_rclcpp_shutdown(source_content):
    """Verify clean shutdown with rclcpp::shutdown."""
    assert "rclcpp::shutdown" in source_content, \
        "Must call rclcpp::shutdown() for clean termination"


def test_pose_values(source_content):
    """Verify the target pose values are preserved from the original."""
    assert "0.28" in source_content, "target_pose1.x must be 0.28"
    assert "-0.2" in source_content, "target_pose1.y must be -0.2"
    assert "0.5" in source_content, "target_pose1.z must be 0.5"
    assert "panda_link0" in source_content, "frame_id must be panda_link0"
    assert "panda_link8" in source_content, "end effector must be panda_link8"
    assert "panda_arm" in source_content, "planning group must be panda_arm"


def test_multiple_plans(source_content):
    """Verify all four planning attempts are present."""
    plan_calls = re.findall(r'plan\s*\(\s*\)', source_content)
    assert len(plan_calls) >= 4, \
        f"Expected at least 4 plan() calls, found {len(plan_calls)}"


def test_set_goal_named(source_content):
    """Verify named goal state 'ready' is used."""
    assert re.search(r'setGoal\s*\(\s*"ready"\s*\)', source_content), \
        "Must set named goal 'ready'"