import pytest
import sys
import os
import re
import time
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

# Ensure the package root (where TaskSmach.py lives) is on the path
PACKAGE_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PACKAGE_ROOT))


@pytest.fixture
def code():
    """Read the source code for static checks."""
    with open(PACKAGE_ROOT / "TaskSmach.py", 'r') as f:
        return f.read()


# ============================================================
# Static oracle tests (must keep passing)
# ============================================================

def test_concurrency_architecture(code):
    concurrency_keywords = ["MultiThreadedExecutor", "threading.Thread", "Future"]
    assert any(word in code for word in concurrency_keywords), \
        "Architecture Error: Task 004 requires a concurrency mechanism."


def test_outcome_logic_mapping(code):
    expected_outcomes = ["TASK_COMPLETED", "TASK_FAILED", "TASK_TIMEOUT"]
    for outcome in expected_outcomes:
        assert outcome in code, f"Logic Error: Missing {outcome}."


def test_preemption_handling(code):
    assert "request_preempt" in code, \
        "Safety Error: The run() method must trigger preemption on system shutdown."


def test_introspection_node_handle(code):
    assert re.search(r"IntrospectionServer\(.*self\.node", code), \
        "API Error: IntrospectionServer must be passed the 'self.node' handle."


def test_strict_style_no_hints(code):
    assert not re.search(r":\s*(Node|int|str|Any|List|Dict)", code), \
        "Style Violation: Type hints are strictly forbidden."


def test_ros2_handle_usage(code):
    assert "self.node.get_logger()" in code, "API Error: Use self.node.get_logger() for logging."
    assert "rospy" not in code, "Legacy Error: Detected 'rospy' in the migrated code."


def test_rclpy_init_protection(code):
    protection_pattern = r"if\s+not\s+rclpy\.ok\(\):\s*(?:\n\s+)*rclpy\.init"
    assert re.search(protection_pattern, code), \
        "Architecture Error: rclpy.init() called without safety check."


# ============================================================
# Runtime tests - actually import and exercise the translated code
# ============================================================

def test_task_state_execute_completed():
    """Test that TaskState.execute returns TASK_COMPLETED on success."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_completed')

        from TaskSmach import TaskState, TaskStatus

        mi = MagicMock()
        mi.is_shutdown.return_value = False
        mi.node = node

        tc = MagicMock()
        mock_task_def = MagicMock()
        mock_task_def.start.return_value = 42
        tc.tasklist = {'TestTask': mock_task_def}
        tc.waitTask.return_value = None

        state = TaskState(mi, tc, 'TestTask', foreground=True)
        result = state.execute(None)

        assert result == 'TASK_COMPLETED', f"Expected TASK_COMPLETED, got {result}"
        mock_task_def.start.assert_called_once()
        tc.waitTask.assert_called_once_with(42)
    finally:
        if node:
            node.destroy_node()


def test_task_state_execute_timeout():
    """Test that TaskState.execute returns TASK_TIMEOUT on timeout exception."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_timeout')

        from TaskSmach import TaskState, TaskException, TaskStatus

        mi = MagicMock()
        mi.is_shutdown.return_value = False
        mi.node = node

        tc = MagicMock()
        mock_task_def = MagicMock()
        mock_task_def.start.return_value = 99
        tc.tasklist = {'TimeoutTask': mock_task_def}
        tc.waitTask.side_effect = TaskException("timeout", id=99, status=TaskStatus.TASK_TIMEOUT)

        state = TaskState(mi, tc, 'TimeoutTask', foreground=True)
        result = state.execute(None)

        assert result == 'TASK_TIMEOUT', f"Expected TASK_TIMEOUT, got {result}"
    finally:
        if node:
            node.destroy_node()


def test_task_state_execute_failed():
    """Test that TaskState.execute returns TASK_FAILED on generic failure."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_failed')

        from TaskSmach import TaskState, TaskException, TaskStatus

        mi = MagicMock()
        mi.is_shutdown.return_value = False
        mi.node = node

        tc = MagicMock()
        mock_task_def = MagicMock()
        mock_task_def.start.return_value = 77
        tc.tasklist = {'FailTask': mock_task_def}
        tc.waitTask.side_effect = TaskException("failed", id=77, status=TaskStatus.TASK_FAILED)

        state = TaskState(mi, tc, 'FailTask', foreground=True)
        result = state.execute(None)

        assert result == 'TASK_FAILED', f"Expected TASK_FAILED, got {result}"
    finally:
        if node:
            node.destroy_node()


def test_task_state_execute_interrupted():
    """Test that TaskState.execute returns TASK_INTERRUPTED on interruption."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_interrupted')

        from TaskSmach import TaskState, TaskException, TaskStatus

        mi = MagicMock()
        mi.is_shutdown.return_value = False
        mi.node = node

        tc = MagicMock()
        mock_task_def = MagicMock()
        mock_task_def.start.return_value = 55
        tc.tasklist = {'InterruptTask': mock_task_def}
        tc.waitTask.side_effect = TaskException("interrupted", id=55, status=TaskStatus.TASK_INTERRUPTED)

        state = TaskState(mi, tc, 'InterruptTask', foreground=True)
        result = state.execute(None)

        assert result == 'TASK_INTERRUPTED', f"Expected TASK_INTERRUPTED, got {result}"
    finally:
        if node:
            node.destroy_node()


def test_task_state_execute_condition_exception():
    """Test that TaskConditionException maps to TASK_INTERRUPTED."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_condition')

        from TaskSmach import TaskState, TaskConditionException

        mi = MagicMock()
        mi.is_shutdown.return_value = False
        mi.node = node

        tc = MagicMock()
        mock_task_def = MagicMock()
        mock_task_def.start.return_value = 33
        tc.tasklist = {'CondTask': mock_task_def}
        tc.waitTask.side_effect = TaskConditionException("condition met", conds=[])

        state = TaskState(mi, tc, 'CondTask', foreground=True)
        result = state.execute(None)

        assert result == 'TASK_INTERRUPTED', f"Expected TASK_INTERRUPTED, got {result}"
    finally:
        if node:
            node.destroy_node()


def test_task_state_shutdown_returns_mission_completed():
    """Test that when shutdown is requested, execute returns MISSION_COMPLETED."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_task_state_shutdown')

        from TaskSmach import TaskState

        mi = MagicMock()
        mi.is_shutdown.return_value = True
        mi.node = node

        tc = MagicMock()

        state = TaskState(mi, tc, 'ShutdownTask', foreground=True)
        result = state.execute(None)

        assert result == 'MISSION_COMPLETED', f"Expected MISSION_COMPLETED, got {result}"
    finally:
        if node:
            node.destroy_node()


def test_request_preempt_calls_stop():
    """Test that request_preempt calls tc.stopTask with the task id."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_preempt_stop')

        from TaskSmach import TaskState

        mi = MagicMock()
        mi.node = node
        tc = MagicMock()

        state = TaskState(mi, tc, 'PreemptTask', foreground=True)
        state.id = 123
        state.request_preempt()

        tc.stopTask.assert_called_once_with(123)
    finally:
        if node:
            node.destroy_node()


def test_signal_handler_sets_shutdown_and_preempts():
    """Test that the signal_handler class sets shutdown and calls request_preempt."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_signal_handler')

        from TaskSmach import MissionStateMachine

        mi = MagicMock()
        mi.shutdown_requested = False
        sm = MagicMock()

        handler = MissionStateMachine.signal_handler(mi, sm)
        handler(2, None)

        assert mi.shutdown_requested is True
        sm.request_preempt.assert_called_once()
    finally:
        if node:
            node.destroy_node()


def test_mission_state_machine_init_creates_node():
    """Test that MissionStateMachine can be instantiated with a mock tc."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_msm_init')

        from TaskSmach import MissionStateMachine

        tc = MagicMock()
        msm = MissionStateMachine(tc=tc, node=node)

        assert msm.tc is tc
        assert msm.node is node
        assert not msm.shutdown_requested
        assert 'TASK_COMPLETED' in msm.default_outcomes
        assert 'TASK_FAILED' in msm.default_outcomes
        assert 'TASK_TIMEOUT' in msm.default_outcomes
        assert 'TASK_INTERRUPTED' in msm.default_outcomes
        assert 'MISSION_COMPLETED' in msm.default_outcomes
    finally:
        if node:
            node.destroy_node()


def test_get_label_generates_unique_labels():
    """Test that getLabel generates unique sequential labels."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = None
    try:
        node = rclpy.create_node('test_get_label')

        from TaskSmach import MissionStateMachine

        tc = MagicMock()
        msm = MissionStateMachine(tc=tc, node=node)

        label1 = msm.getLabel("GoTo")
        label2 = msm.getLabel("GoTo")
        label3 = msm.getLabel("Wait")

        assert label1 != label2, "Labels should be unique"
        assert "GoTo" in label1
        assert "GoTo" in label2
        assert "Wait" in label3
    finally:
        if node:
            node.destroy_node()