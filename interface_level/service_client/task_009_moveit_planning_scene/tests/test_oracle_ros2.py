# src/task_009_moveit_planning_scene/test/test_oracle_ros2.py
# Oracle: static pattern matching only (regex + string search). No compilation/runtime.

import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "apply_planning.cpp"


def read_file(path: Path) -> str:
    if not path.exists():
        raise AssertionError(f"Expected C++ file not found: {path}")
    return path.read_text(encoding="utf-8", errors="ignore")


def _re_search(pattern: str, text: str, flags=re.MULTILINE | re.DOTALL) -> bool:
    try:
        return re.search(pattern, text, flags) is not None
    except re.error as e:
        raise AssertionError(f"Oracle regex invalid:\n{pattern}\nRegex error: {e}")


def assert_has(pattern: str, text: str, msg: str):
    if not _re_search(pattern, text):
        raise AssertionError(msg + f"\nMissing pattern: {pattern}\nFile: {CPP_FILE}")


def assert_not_has(pattern: str, text: str, msg: str):
    if _re_search(pattern, text):
        raise AssertionError(msg + f"\nFound forbidden pattern: {pattern}\nFile: {CPP_FILE}")


def extract_function_body(code: str, func_name: str) -> str:
    m = re.search(rf'\b{re.escape(func_name)}\s*\([^)]*\)\s*\{{', code, re.MULTILINE | re.DOTALL)
    if not m:
        return ""
    i = m.end()
    depth = 1
    while i < len(code) and depth > 0:
        c = code[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
        i += 1
    return code[m.start():i] if depth == 0 else ""


# ----------------------------
# Category 1: CONTEXT
# ----------------------------

def test_category_1_context_ros2_moveit_planning_scene_publisher():
    code = read_file(CPP_FILE)

    # ROS2 present
    assert_has(r'#include\s*<\s*rclcpp/[^>]+>', code,
               "[CONTEXT] Expected ROS2 include '#include <rclcpp/...>'.")

    # ROS1 absent
    for pat, msg in [
        (r'#include\s*<\s*ros/ros\.h\s*>', "[CONTEXT] ROS1 header '<ros/ros.h>' must not appear."),
        (r'\bros::NodeHandle\b', "[CONTEXT] ROS1 ros::NodeHandle must not appear."),
        (r'\bros::init\s*\(', "[CONTEXT] ROS1 ros::init must not appear."),
        (r'\bROS_(INFO|WARN|ERROR|DEBUG|FATAL)\b', "[CONTEXT] ROS1 ROS_* macros must not appear."),
    ]:
        assert_not_has(pat, code, msg)

    # MoveIt messages
    assert_has(r'#include\s*<\s*moveit_msgs/msg/planning_scene\.hpp\s*>', code,
               "[CONTEXT] Expected MoveIt PlanningScene include.")
    assert_has(r'#include\s*<\s*moveit_msgs/msg/collision_object\.hpp\s*>', code,
               "[CONTEXT] Expected MoveIt CollisionObject include.")
    assert_has(r'#include\s*<\s*moveit_msgs/msg/attached_collision_object\.hpp\s*>', code,
               "[CONTEXT] Expected MoveIt AttachedCollisionObject include.")

    # planning_scene publisher
    assert_has(
        r'create_publisher\s*<\s*moveit_msgs::msg::PlanningScene\s*>\s*\(\s*["\']planning_scene["\']\s*,',
        code,
        "[CONTEXT] Expected publisher: create_publisher<moveit_msgs::msg::PlanningScene>(\"planning_scene\", ...).",
    )


# ----------------------------
# Category 2: CORE TRANSITION (world -> attached)
# ----------------------------

def test_category_2_core_transition_attach_step_world_remove_and_robot_attach():
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "step_attach_object_and_remove_from_world__TODO")
    if not body:
        raise AssertionError("[CORE] Missing function step_attach_object_and_remove_from_world__TODO(...) body.")

    # must construct planning_scene
    assert_has(
        r'moveit_msgs::msg::PlanningScene\s+planning_scene\s*;',
        body,
        "[CORE] Expected 'moveit_msgs::msg::PlanningScene planning_scene;' in attach step.",
    )

    # must have world removal (CollisionObject + REMOVE + push to world)
    assert_has(r'moveit_msgs::msg::CollisionObject\s+\w+\s*;', body,
               "[CORE] Expected creating a CollisionObject in attach step (for world removal).")
    assert_has(r'\.\s*operation\s*=\s*moveit_msgs::msg::CollisionObject::REMOVE\s*;', body,
               "[CORE] Expected operation = CollisionObject::REMOVE in attach step.")
    assert_has(r'planning_scene\s*\.\s*world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(', body,
               "[CORE] Expected pushing remove-object into planning_scene.world.collision_objects.")

    # must attach to robot state using the provided attached_object
    assert_has(
        r'planning_scene\s*\.\s*robot_state\s*\.\s*attached_collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\)\s*;',
        body,
        "[CORE] Expected planning_scene.robot_state.attached_collision_objects.push_back(attached_object).",
    )

    # must publish and prompt (tutorial boundary)
    assert_has(r'(publish_scene_diff\s*\(|pub\s*->\s*publish\s*\()', body,
               "[CORE] Expected publishing the PlanningScene diff in attach step.")
    assert_has(r'visual_tools\s*\.\s*prompt\s*\(', body,
               "[CORE] Expected visual_tools.prompt(...) in attach step.")


# ----------------------------
# Category 3: MINIMAL DIFF HYGIENE
# ----------------------------

def test_category_3_minimal_diff_hygiene_reset_and_not_keep_in_world():
    code = read_file(CPP_FILE)
    body = extract_function_body(code, "step_attach_object_and_remove_from_world__TODO")
    if not body:
        raise AssertionError("[HYGIENE] Missing attach step body.")

    # Require robot_state.is_diff = true
    assert_has(
        r'planning_scene\s*\.\s*robot_state\s*\.\s*is_diff\s*=\s*true\s*;',
        body,
        "[HYGIENE] Expected planning_scene.robot_state.is_diff = true; (required for attached diffs).",
    )

    # Require explicit reset of BOTH containers (hard, discriminative)
    world_reset = (
        _re_search(r'world\s*\.\s*collision_objects\s*\.\s*clear\s*\(\s*\)\s*;', body) or
        _re_search(r'world\s*\.\s*collision_objects\s*=\s*\{', body) or
        _re_search(r'world\s*\.\s*collision_objects\s*=\s*std::vector', body)
    )
    robot_reset = (
        _re_search(r'robot_state\s*\.\s*attached_collision_objects\s*\.\s*clear\s*\(\s*\)\s*;', body) or
        _re_search(r'robot_state\s*\.\s*attached_collision_objects\s*=\s*\{', body) or
        _re_search(r'robot_state\s*\.\s*attached_collision_objects\s*=\s*std::vector', body)
    )

    if not world_reset:
        raise AssertionError(
            "[HYGIENE] Attach step must reset planning_scene.world.collision_objects "
            "(clear() or overwrite) to ensure minimal diff."
        )
    if not robot_reset:
        raise AssertionError(
            "[HYGIENE] Attach step must reset planning_scene.robot_state.attached_collision_objects "
            "(clear() or overwrite) to ensure minimal diff."
        )

    # Forbid keeping object in world in the same attach step
    assert_not_has(
        r'world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\.\s*object\s*\)',
        body,
        "[HYGIENE] Attach step must not push attached_object.object into world.collision_objects (world -> attached).",
    )


# ----------------------------
# Category 4: DUALITY (detach is inverse)
# ----------------------------

def test_category_4_duality_detach_step_inverse_transition():
    code = read_file(CPP_FILE)
    detach = extract_function_body(code, "step_detach_object_and_return_to_world")
    if not detach:
        raise AssertionError("[DUALITY] Missing function step_detach_object_and_return_to_world(...) body.")

    # detach must remove from robot
    assert_has(
        r'\.\s*object\s*\.\s*operation\s*=\s*moveit_msgs::msg::CollisionObject::REMOVE\s*;',
        detach,
        "[DUALITY] Detach step must set attached object's operation to CollisionObject::REMOVE.",
    )
    assert_has(
        r'robot_state\s*\.\s*attached_collision_objects\s*\.\s*push_back\s*\(',
        detach,
        "[DUALITY] Detach step must push a detach message into robot_state.attached_collision_objects.",
    )

    # detach must return object to world (tutorial-shaped)
    assert_has(
        r'world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\.\s*object\s*\)\s*;',
        detach,
        "[DUALITY] Detach step must return the object to the world: world.collision_objects.push_back(attached_object.object).",
    )
