"""
Runtime test for task_009_moveit_planning_scene.

We launch the compiled C++ node (apply_planning) with a test subscriber on
the 'planning_scene' topic to verify it actually publishes PlanningScene
diff messages with the correct structure.

The node waits for at least one subscriber before publishing, so our
subscriber satisfies that requirement. The node also calls visual_tools.prompt()
which in our stub just logs and continues, so the node runs through all steps
automatically.
"""

import subprocess
import time
import threading
import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PlanningSceneCollector(Node):
    """Subscribes to 'planning_scene' and collects messages."""

    def __init__(self):
        super().__init__('test_planning_scene_collector')
        self.messages = []
        self.lock = threading.Lock()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100,
        )
        # Import the local message type
        from task_009_moveit_planning_scene.msg import PlanningScene
        self.sub = self.create_subscription(
            PlanningScene, 'planning_scene', self._cb, qos
        )

    def _cb(self, msg):
        with self.lock:
            self.messages.append(msg)
            self.get_logger().info(
                f"Received PlanningScene msg #{len(self.messages)}: "
                f"world_cos={len(msg.world.collision_objects)}, "
                f"attached={len(msg.robot_state.attached_collision_objects)}, "
                f"is_diff={msg.is_diff}"
            )

    def get_messages(self):
        with self.lock:
            return list(self.messages)


def spin_node(node, stop_event):
    """Spin the node until stop_event is set."""
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.02)


class TestApplyPlanningRuntime:
    """Launch the real apply_planning node and verify published messages."""

    def test_planning_scene_messages(self):
        rclpy.init()
        collector = PlanningSceneCollector()
        stop_event = threading.Event()
        spin_thread = threading.Thread(target=spin_node, args=(collector, stop_event), daemon=True)
        spin_thread.start()

        from task_009_moveit_planning_scene.msg import CollisionObject

        proc = None
        try:
            # Give the subscriber node a moment to be fully ready
            time.sleep(0.5)

            # Launch the compiled node
            proc = subprocess.Popen(
                ['ros2', 'run', 'task_009_moveit_planning_scene', 'apply_planning'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Wait for the node to finish (it auto-continues through prompts)
            # Timeout after 30 seconds
            deadline = time.time() + 30.0
            while time.time() < deadline:
                ret = proc.poll()
                if ret is not None:
                    break
                time.sleep(0.2)

            # If still running, kill it
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)

            # Give time for any remaining messages to arrive
            time.sleep(2.0)

            msgs = collector.get_messages()

            # We expect at least 4 planning scene messages (add, attach, detach, remove)
            assert len(msgs) >= 4, (
                f"Expected at least 4 PlanningScene messages, got {len(msgs)}"
            )

            # All messages should be diffs
            for i, m in enumerate(msgs):
                assert m.is_diff, f"Message {i} should have is_diff=True"

            # Step 1: ADD object to world (CollisionObject with ADD operation)
            found_add = False
            for m in msgs:
                for co in m.world.collision_objects:
                    if co.id == "box" and co.operation == CollisionObject.ADD:
                        found_add = True
                        break
            assert found_add, "Step 1: expected CollisionObject ADD in world"

            # Step 2: REMOVE from world + attach to robot
            found_step2 = False
            for m in msgs:
                has_remove = False
                has_attach = False
                for co in m.world.collision_objects:
                    if co.id == "box" and co.operation == CollisionObject.REMOVE:
                        has_remove = True
                for aco in m.robot_state.attached_collision_objects:
                    if aco.object.id == "box" and aco.object.operation == CollisionObject.ADD:
                        has_attach = True
                if has_remove and has_attach:
                    found_step2 = True
                    assert m.robot_state.is_diff, "Step 2: robot_state.is_diff should be True"
                    break
            assert found_step2, "Step 2: expected CollisionObject REMOVE in world + attached object"

            # Step 3: Detach from robot (REMOVE in attached) + return to world (ADD)
            found_detach = False
            for m in msgs:
                has_detach = False
                has_return = False
                for aco in m.robot_state.attached_collision_objects:
                    if aco.object.id == "box" and aco.object.operation == CollisionObject.REMOVE:
                        has_detach = True
                for co in m.world.collision_objects:
                    if co.id == "box" and co.operation == CollisionObject.ADD:
                        has_return = True
                if has_detach and has_return:
                    found_detach = True
                    break
            assert found_detach, "Step 3: expected detach + return to world"

            # Step 4: Final REMOVE from world
            found_final_remove = False
            for m in msgs:
                if len(m.world.collision_objects) >= 1:
                    for co in m.world.collision_objects:
                        if co.id == "box" and co.operation == CollisionObject.REMOVE:
                            if len(m.robot_state.attached_collision_objects) == 0:
                                found_final_remove = True
                                break
            assert found_final_remove, "Step 4: expected final REMOVE from world"

        finally:
            stop_event.set()
            spin_thread.join(timeout=3)
            collector.destroy_node()
            rclpy.shutdown()
            if proc and proc.poll() is None:
                proc.kill()
                proc.wait(timeout=3)


class TestApplyPlanningCpp:
    """Source-level checks on the translated C++ file."""

    @staticmethod
    def _read():
        from pathlib import Path
        p = Path(__file__).resolve().parent / "apply_planning.cpp"
        assert p.exists(), f"apply_planning.cpp not found at {p}"
        return p.read_text()

    @staticmethod
    def _extract_body(code, func_name):
        import re
        m = re.search(
            rf'\b{re.escape(func_name)}\s*\([^)]*\)\s*\{{',
            code, re.DOTALL,
        )
        if not m:
            return ""
        i = m.end()
        depth = 1
        while i < len(code) and depth > 0:
            if code[i] == "{":
                depth += 1
            elif code[i] == "}":
                depth -= 1
            i += 1
        return code[m.start():i] if depth == 0 else ""

    def test_ros2_includes(self):
        import re
        code = self._read()
        assert re.search(r'#include\s*<\s*rclcpp/rclcpp\.hpp\s*>', code)

    def test_no_ros1_remnants(self):
        import re
        code = self._read()
        assert not re.search(r'#include\s*<\s*ros/ros\.h\s*>', code)
        assert not re.search(r'\bros::init\s*\(', code)
        assert not re.search(r'\bros::NodeHandle\b', code)

    def test_moveit_msg_includes(self):
        import re
        code = self._read()
        # The code includes local equivalents that mirror moveit_msgs/msg/ paths
        # Check for the planning_scene, collision_object, attached_collision_object includes
        assert re.search(r'#include\s*<\s*\S*planning_scene\.hpp\s*>', code)
        assert re.search(r'#include\s*<\s*\S*collision_object\.hpp\s*>', code)
        assert re.search(r'#include\s*<\s*\S*attached_collision_object\.hpp\s*>', code)
        # Also check the namespace aliases exist
        assert re.search(r'moveit_msgs::msg::PlanningScene', code)
        assert re.search(r'moveit_msgs::msg::CollisionObject', code)
        assert re.search(r'moveit_msgs::msg::AttachedCollisionObject', code)

    def test_planning_scene_publisher(self):
        import re
        code = self._read()
        assert re.search(
            r'create_publisher\s*<\s*moveit_msgs::msg::PlanningScene\s*>\s*\(\s*["\']planning_scene["\']\s*,',
            code,
        )

    def test_attach_function_body(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert body, "Function not found"
        assert "planning_scene" in body

    def test_attach_creates_planning_scene(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'moveit_msgs::msg::PlanningScene\s+planning_scene\s*;', body)

    def test_attach_creates_collision_object_remove(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'moveit_msgs::msg::CollisionObject\s+\w+\s*;', body)
        assert re.search(r'\.\s*operation\s*=\s*moveit_msgs::msg::CollisionObject::REMOVE\s*;', body)

    def test_attach_pushes_remove_to_world(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'planning_scene\s*\.\s*world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(', body)

    def test_attach_pushes_attached_object_to_robot_state(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(
            r'planning_scene\s*\.\s*robot_state\s*\.\s*attached_collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\)\s*;',
            body,
        )

    def test_attach_robot_state_is_diff(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'planning_scene\s*\.\s*robot_state\s*\.\s*is_diff\s*=\s*true\s*;', body)

    def test_attach_clears_world_collision_objects(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'world\s*\.\s*collision_objects\s*\.\s*clear\s*\(\s*\)\s*;', body)

    def test_attach_clears_attached_collision_objects(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'robot_state\s*\.\s*attached_collision_objects\s*\.\s*clear\s*\(\s*\)\s*;', body)

    def test_attach_publishes(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'(publish_scene_diff\s*\(|pub\s*->\s*publish\s*\()', body)

    def test_attach_prompts(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert re.search(r'visual_tools\s*\.\s*prompt\s*\(', body)

    def test_attach_does_not_add_object_back_to_world(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_attach_object_and_remove_from_world__TODO")
        assert not re.search(
            r'world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\.\s*object\s*\)',
            body,
        )

    def test_detach_function_exists(self):
        code = self._read()
        body = self._extract_body(code, "step_detach_object_and_return_to_world")
        assert body

    def test_detach_removes_from_robot(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_detach_object_and_return_to_world")
        assert re.search(
            r'\.\s*object\s*\.\s*operation\s*=\s*moveit_msgs::msg::CollisionObject::REMOVE\s*;', body
        )

    def test_detach_returns_to_world(self):
        import re
        code = self._read()
        body = self._extract_body(code, "step_detach_object_and_return_to_world")
        assert re.search(
            r'world\s*\.\s*collision_objects\s*\.\s*push_back\s*\(\s*attached_object\s*\.\s*object\s*\)\s*;',
            body,
        )

    def test_main_uses_rclcpp_init_and_shutdown(self):
        import re
        code = self._read()
        assert re.search(r'rclcpp::init\s*\(', code)
        assert re.search(r'rclcpp::shutdown\s*\(', code)