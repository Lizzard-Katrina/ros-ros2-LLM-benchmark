import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "code.py"

@pytest.fixture
def code():
    with open(PY_FILE, 'r') as f:
        return f.read()

# 1. NEW: Check Constructor Argument Order (Caught the previous error)
def test_constructor_argument_order(code):
    """Verify that 'node' comes before 'model_name' as per TODO."""
    # Matches: def __init__(self, node, model_name, ...
    pattern = r"def\s+__init__\s*\(\s*self\s*,\s*node\s*,\s*model_name"
    assert re.search(pattern, code), "Order Violation: Constructor must be (self, node, model_name)."

# 2. Check Clock and Manual Duration Logic (Strictly enforced now)
def test_clock_and_manual_math(code):
    """Verify manual conversion from nanoseconds to seconds."""
    # Matches: .nanoseconds / 1e9 or .nanoseconds / 1000000000
    nano_conv = r"\.nanoseconds\s*/\s*1(?:e9|000000000)"
    assert re.search(nano_conv, code), \
        "Logic/Style Violation: Must manually convert nanoseconds to seconds for comparison as per TODO."
    # Ensure they didn't cheat by using the Duration class
    assert "Duration(seconds=" not in code, "Constraint Violation: Duration object usage is forbidden in this task."

# 3. NEW: Service Future Handling (Verification of async logic)
def test_service_future_logic(code):
    """Verify the use of futures and spin_until_future_complete."""
    assert "call_async" in code, "Missing async service call."
    assert "spin_until_future_complete" in code, "Missing future completion handling."
    assert "future.result()" in code, "Must check the result of the future."

# 4. Executor Spinning in Loop
def test_executor_spinning_in_loop(code):
    """Verify rclpy.spin_once is used within the while loop."""
    pattern = r"while.*:\s*(?:.|\n)*?rclpy\.spin_once\s*\(\s*self\.node"
    assert re.search(pattern, code, re.DOTALL), "Missing rclpy.spin_once() inside the wait loop."

# 5. Preemption Check
def test_preemption_check_retention(code):
    """Ensure the state remains responsive to SMACH preemption."""
    assert "self.preempt_requested()" in code, "Preemption check was lost during migration."

# 6. Logging Migration (No rospy)
def test_logging_migration(code):
    """Verify transition from rospy.log to node.get_logger()."""
    assert "rospy" not in code, "Legacy rospy symbols detected."
    assert "self.node.get_logger()" in code, "Must use self.node.get_logger()."

# 7. No Type Hints (Strict Style)
def test_no_type_hints(code):
    """Ensure no type hints like ': Node' are present."""
    assert not re.search(r"node\s*:\s*\w+", code), "Style Violation: Type hints are strictly forbidden."
