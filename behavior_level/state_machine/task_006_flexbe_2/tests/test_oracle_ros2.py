import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "get_joint_values_state.py"

@pytest.fixture
def code():
    with open(PY_FILE, 'r') as f:
        return f.read()

def test_strict_ros2_clock_usage(code):
    """FAIL if 'Clock()' appears anywhere in the file (even as an import or variable)."""
    # Even if they do self._clock = Clock(), this should kill the test.
    assert "Clock()" not in code, "Architecture Error: Manual Clock instantiation is forbidden. Use self.get_clock()."
    assert "self.get_clock().now()" in code, "API Error: Node clock must be used for sim_time compatibility."

def test_true_buffer_drain_logic(code):
    """
    FAIL if the buffer is not drained using a WHILE loop. 
    Many LLMs use 'if' which only processes one message per tick.
    """
    # Regex to find 'while' followed by 'has_buffered'
    while_pattern = r"while\s+.*has_buffered"
    assert re.search(while_pattern, code), \
        "Logic Error: You must use a WHILE loop to drain ALL messages in the buffer per tick."

def test_incremental_update_logic(code):
    """
    FAIL if the LLM doesn't check if a joint value is 'None' before updating.
    This is critical for collecting data across multiple partial messages.
    """
    # It must check if the current value in self._joint_values is None
    assert "is None" in code or "== None" in code, \
        "Logic Error: Must check if a joint value is None before assigning to avoid overwriting with stale data."

def test_dynamic_index_resolution(code):
    """Verify names are resolved to indices within the message structure."""
    assert "msg.name.index" in code or "zip(" in code or "dict(zip" in code, \
        "Logic Error: Failed to implement dynamic name-to-index resolution."
    # Ban hardcoded positions again
    assert not re.search(r"position\[\d+\]", code), "Safety Error: Hardcoded indexing detected."

def test_lifecycle_on_enter_clean(code):
    """Ensure proper reset and enablement."""
    assert "enable_buffer" in code, "Lifecycle Error: enable_buffer() must be called."
    # Check for [None] * len(...) pattern to ensure it supports arbitrary joint counts
    assert "None" in code and "len(" in code, \
        "Initialization Error: self._joint_values must be initialized as a list of Nones."
