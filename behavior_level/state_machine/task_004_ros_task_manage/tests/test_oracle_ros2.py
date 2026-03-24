import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "TaskSmach.py"

@pytest.fixture
def code():
    with open(PY_FILE, 'r') as f:
        return f.read()

# 1. Threading/Executor Logic (The "Killer" test for ROS 2)
def test_concurrency_architecture(code):
    """
    ROS 2 requires an Executor (often MultiThreaded) to process TaskClient callbacks 
    while the SMACH thread is blocked on 'execute'.
    """
    # LLM must use some form of threading or MultiThreadedExecutor
    concurrency_keywords = ["MultiThreadedExecutor", "threading.Thread", "Future"]
    assert any(word in code for word in concurrency_keywords), \
        "Architecture Error: Task 004 requires a concurrency mechanism to avoid blocking the ROS 2 node."

# 2. Outcome Mapping Integrity
def test_outcome_logic_mapping(code):
    """Checks if the LLM remembered to map TaskStatus to SMACH outcomes."""
    expected_outcomes = ["TASK_COMPLETED", "TASK_FAILED", "TASK_TIMEOUT"]
    for outcome in expected_outcomes:
        assert outcome in code, f"Logic Error: Missing {outcome} in state execution logic."

# 3. Shutdown and Preemption (Fidelity test)
def test_preemption_handling(code):
    """Verify that on SIGINT/Shutdown, the code calls request_preempt()."""
    # This ensures the TaskManager actually stops the remote tasks on Ctrl-C
    assert "request_preempt" in code, \
        "Safety Error: The run() method must trigger preemption on system shutdown."

# 4. Introspection Server Node Injection
def test_introspection_node_handle(code):
    """In ROS 2, smach_ros.IntrospectionServer requires a node handle."""
    assert re.search(r"IntrospectionServer\(.*self\.node", code), \
        "API Error: IntrospectionServer must be passed the 'self.node' handle."

# 5. Type Hint Restriction (Strict Style)
def test_strict_style_no_hints(code):
    """Check for absence of ': Node', ': int', etc."""
    # Matches common type hint patterns
    assert not re.search(r":\s*(Node|int|str|Any|List|Dict)", code), \
        "Style Violation: Type hints are strictly forbidden in this benchmark."

# 6. Logging and Handle Usage
def test_ros2_handle_usage(code):
    """Ensure self.node is used instead of legacy rospy."""
    assert "self.node.get_logger()" in code, "API Error: Use self.node.get_logger() for logging."
    assert "rospy" not in code, "Legacy Error: Detected 'rospy' in the migrated code."

def test_rclpy_init_protection(code):
    """
    CRITICAL: In a library like TaskManager, __init__ must not blindly call rclpy.init().
    It must check if rclpy is already active to avoid RuntimeError.
    """
    protection_pattern = r"if\s+not\s+rclpy\.ok\(\):\s*(?:\n\s+)*rclpy\.init"
    assert re.search(protection_pattern, code), \
        "Architecture Error: rclpy.init() called without safety check. This will crash in multi-node systems."
