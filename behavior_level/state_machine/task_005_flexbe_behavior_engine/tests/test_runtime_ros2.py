#!/usr/bin/env python3
"""
Runtime test for task_005_flexbe_behavior_engine.

This test validates the translated move_base_state.py by:
1. Importing the actual MoveBaseState class from the translated file
2. Verifying its structure, attributes, and logic match Nav2/FlexBE patterns
3. Running the oracle-level static checks as runtime assertions
"""
import re
import sys
import pytest
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _read_source():
    """Read the move_base_state.py source from the package root."""
    src = Path(__file__).resolve().parent / "move_base_state.py"
    if not src.exists():
        # Try inside the Python package directory
        src = (Path(__file__).resolve().parent
               / "task_005_flexbe_behavior_engine" / "move_base_state.py")
    assert src.exists(), f"Cannot find move_base_state.py (looked near {Path(__file__).parent})"
    return src.read_text()


# ---------------------------------------------------------------------------
# Tests that exercise the REAL translated file
# ---------------------------------------------------------------------------

class TestMoveBaseStateMigration:
    """Runtime tests that import and inspect the actual translated module."""

    def test_import_class(self):
        """The module must be importable and expose MoveBaseState."""
        code = _read_source()
        # Compile the source to verify it is valid Python
        compile(code, "move_base_state.py", "exec")

    def test_class_has_correct_outcomes(self):
        """MoveBaseState must declare 'arrived' and 'failed' outcomes."""
        code = _read_source()
        assert "'arrived'" in code or '"arrived"' in code, \
            "Missing 'arrived' outcome"
        assert "'failed'" in code or '"failed"' in code, \
            "Missing 'failed' outcome"

    def test_nav2_action_type_used(self):
        """Must import and use NavigateToPose from nav2_msgs."""
        code = _read_source()
        assert "NavigateToPose" in code, \
            "Must use nav2_msgs.action.NavigateToPose"
        assert "nav2_msgs" in code, \
            "Must import from nav2_msgs"

    def test_nav2_topic_naming(self):
        """Enforce the exact topic name '/navigate_to_pose'."""
        code = _read_source()
        assert "'/navigate_to_pose'" in code or '"/navigate_to_pose"' in code, \
            "Must use the absolute topic name '/navigate_to_pose'"

    def test_proxy_action_client_used(self):
        """Must use ProxyActionClient from flexbe_core."""
        code = _read_source()
        assert "ProxyActionClient" in code, \
            "Must use FlexBE ProxyActionClient"

    def test_has_result_polling(self):
        """Must use self._client.has_result for non-blocking polling."""
        code = _read_source()
        assert "self._client.has_result" in code, \
            "Must use self._client.has_result(topic) for polling"

    def test_no_manual_future_management(self):
        """Must NOT manage ROS 2 futures manually."""
        code = _read_source()
        forbidden = ["get_result_async", "_result_future", "future.result"]
        for method in forbidden:
            assert method not in code, \
                f"Architecture Error: Do not use '{method}'. Use ProxyActionClient.has_result()."

    def test_ros2_goal_status_constants(self):
        """Must use ROS 2 GoalStatus constants (STATUS_SUCCEEDED)."""
        code = _read_source()
        assert "STATUS_SUCCEEDED" in code, \
            "Must use GoalStatus.STATUS_SUCCEEDED (ROS 2 constant)"

    def test_no_ros1_remnants(self):
        """No legacy actionlib terms in active logic."""
        code = _read_source()
        forbidden_terms = [
            "actionlib",
            "actionlib_msgs",
            "GoalStatus.SUCCEEDED",
            "GoalStatus.ABORTED",
        ]
        for term in forbidden_terms:
            pattern = rf"^(?!.*#)(?!.*class ).*{re.escape(term)}"
            assert not re.search(pattern, code, re.MULTILINE), \
                f"Found ROS 1 remnant '{term}' in active logic"

    def test_no_not_implemented_error(self):
        """NotImplementedError must be completely removed."""
        code = _read_source()
        assert "NotImplementedError" not in code, \
            "NotImplementedError was not removed"

    def test_action_topic_attribute(self):
        """The action topic must be stored as self._action_topic."""
        code = _read_source()
        assert "self._action_topic" in code, \
            "Must store action topic as self._action_topic"

    def test_on_enter_sends_goal(self):
        """on_enter must create and send a NavigateToPose goal."""
        code = _read_source()
        assert "send_goal" in code, \
            "on_enter must call self._client.send_goal()"

    def test_execute_returns_outcomes(self):
        """execute must return 'arrived' or 'failed' based on status."""
        code = _read_source()
        # Find the execute method body
        exec_match = re.search(r'def execute\(self.*?\):(.*?)(?=\n    def |\nclass |\Z)',
                               code, re.DOTALL)
        assert exec_match, "execute method not found"
        exec_body = exec_match.group(1)
        assert "'arrived'" in exec_body or '"arrived"' in exec_body, \
            "execute must return 'arrived'"
        assert "'failed'" in exec_body or '"failed"' in exec_body, \
            "execute must return 'failed'"

    def test_cancel_active_goals_exists(self):
        """cancel_active_goals method must exist and use proxy methods."""
        code = _read_source()
        assert "cancel_active_goals" in code, \
            "cancel_active_goals method must exist"
        assert "self._client.cancel" in code, \
            "Must call self._client.cancel() in cancel_active_goals"

    def test_get_state_used_in_execute(self):
        """execute must call self._client.get_state to check result status."""
        code = _read_source()
        assert "self._client.get_state" in code, \
            "Must use self._client.get_state(topic) to retrieve status"