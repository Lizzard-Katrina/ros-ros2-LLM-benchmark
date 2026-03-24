import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "depth_reach.cpp"
FLAGS = re.MULTILINE | re.DOTALL


def _read() -> str:
    assert CPP_FILE.exists(), f"Expected C++ file at: {CPP_FILE}"
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")


def _has(pat: str, s: str) -> bool:
    return re.search(pat, s, FLAGS) is not None


def _assert_has(pat: str, s: str, msg: str):
    if not _has(pat, s):
        raise AssertionError(msg + f"\nMissing pattern:\n{pat}")


def _assert_not_has(pat: str, s: str, msg: str):
    if _has(pat, s):
        raise AssertionError(msg + f"\nForbidden pattern found:\n{pat}")


# 1) ROS2 not ROS1
def test_01_ros2_not_ros1():
    s = _read()
    _assert_has(r'#include\s*[<"]\s*rclcpp/rclcpp\.hpp\s*[>"]', s,
                "Expected ROS2 include rclcpp/rclcpp.hpp.")
    _assert_not_has(r'#include\s*[<"]\s*ros/ros\.h\s*[>"]', s,
                    "Found ROS1 ros/ros.h; expected ROS2-only implementation.")
    _assert_not_has(r"\bros::(init|NodeHandle|spin|AsyncSpinner)\b", s,
                    "Found ROS1 ros:: APIs; expected ROS2 rclcpp APIs.")


# 2) Uses MoveIt2 planning interface types (MoveGroupInterface + PlanningSceneInterface)
def test_02_moveit2_interfaces_present():
    s = _read()
    _assert_has(r"(moveit::planning_interface::MoveGroupInterface|MoveGroupInterface)", s,
                "Expected MoveGroupInterface usage (MoveIt pick/place API).")
    _assert_has(r"(moveit::planning_interface::PlanningSceneInterface|PlanningSceneInterface)", s,
                "Expected PlanningSceneInterface usage (for collision objects).")


# 3) Collision objects: table1/table2/object ids exist
def test_03_collision_object_ids_present():
    s = _read()
    _assert_has(r'"table1"', s, "Expected collision object id/name 'table1'.")
    _assert_has(r'"table2"', s, "Expected collision object id/name 'table2'.")
    _assert_has(r'"object"', s, "Expected collision object id/name 'object' (grasp target).")


# 4) Collision object uses frame panda_link0 (semantic anchor from ROS1 reference)
def test_04_frame_id_panda_link0_present():
    s = _read()
    _assert_has(r'(header\.\s*frame_id\s*=\s*"panda_link0")|("panda_link0")', s,
                "Expected frame_id anchor 'panda_link0' for scene/poses (semantic match to reference).")


# 5) Adds collision objects via planning scene interface (apply/add)
def test_05_planning_scene_apply_collision_objects():
    s = _read()
    _assert_has(r"(applyCollisionObjects|addCollisionObjects)\s*\(", s,
                "Expected PlanningSceneInterface to add/apply collision objects (applyCollisionObjects/addCollisionObjects).")
    _assert_has(r"(moveit_msgs::msg::CollisionObject|CollisionObject)\b", s,
                "Expected CollisionObject message type (MoveIt collision objects).")


# 6) Pick pipeline: constructs grasps vector and calls pick on 'object'
def test_06_pick_pipeline_present():
    s = _read()
    _assert_has(r"(std::vector\s*<\s*moveit_msgs::(msg::)?Grasp\s*>\s*\w+|Grasp\s*\w+\s*;)", s,
                "Expected grasp(s) construction (vector<Grasp> or equivalent).")
    _assert_has(r"\b(pick)\s*\(\s*\"object\"|\bpick\s*\(\s*\"object\"", s,
                "Expected MoveGroupInterface.pick called for 'object'.")


# 7) Pick pipeline: sets support surface name to table1
def test_07_pick_has_table1_as_support_semantics():
    s = _read()
    ok = (
        _has(r'setSupportSurfaceName\s*\(\s*"table1"\s*\)', s) or
        (_has(r'"table1"', s) and _has(r'\bpick\s*\(\s*"object"', s))
    )
    if not ok:
        raise AssertionError(
            "Expected pick to incorporate table1 as the support surface semantics.\n"
            "Acceptable evidence:\n"
            "- setSupportSurfaceName(\"table1\")\n"
            "- OR table1 referenced in code together with a pick(\"object\", ...) call."
        )


# 8) Place pipeline: constructs place location(s) and calls place on 'object'
def test_08_place_pipeline_present():
    s = _read()
    _assert_has(r"(std::vector\s*<\s*moveit_msgs::(msg::)?PlaceLocation\s*>\s*\w+|PlaceLocation\s*\w+\s*;)", s,
                "Expected place location(s) construction (vector<PlaceLocation> or equivalent).")
    _assert_has(r"\b(place)\s*\(\s*\"object\"|\bplace\s*\(\s*\"object\"", s,
                "Expected MoveGroupInterface.place called for 'object'.")


# 9) Place pipeline: sets support surface name to table2
def test_09_place_has_table2_as_support_semantics():
    s = _read()
    ok = (
        _has(r'setSupportSurfaceName\s*\(\s*"table2"\s*\)', s) or
        (_has(r'"table2"', s) and _has(r'\bplace\s*\(\s*"object"', s))
    )
    if not ok:
        raise AssertionError(
            "Expected place to incorporate table2 as the support surface semantics.\n"
            "Acceptable evidence:\n"
            "- setSupportSurfaceName(\"table2\")\n"
            "- OR table2 referenced in code together with a place(\"object\", ...) call."
        )
