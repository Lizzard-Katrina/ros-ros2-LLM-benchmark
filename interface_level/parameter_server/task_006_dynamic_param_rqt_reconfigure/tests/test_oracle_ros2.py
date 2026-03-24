import re
import pytest
from pathlib import Path

# Assuming the file is saved as param_api.py in the src directory
PY_FILE = Path(__file__).resolve().parents[1] / "param_api.py"

@pytest.fixture
def code_content():
    with open(PY_FILE, "r", encoding="utf-8") as f:
        # Remove comments to focus on actual code logic
        return re.sub(r'#.*', '', f.read())

# --- 1. Deadlock Prevention ---
def test_deadlock_avoidance(code_content):
    """Checks if the model correctly avoids client.call() which deadlocks rclpy."""
    assert "call_async" in code_content, "Critical: Must use call_async to bridge sync/async."
    assert ".call(" not in code_content, "Failing: client.call() will deadlock the rqt executor."

# --- 2. Synchronization Primitive Selection ---
def test_sync_primitive(code_content):
    """Verify if the model picks the correct threading tool (Event) used in rqt_reconfigure."""
    # Models might try to use time.sleep() (wrong) or Condition (too complex)
    assert "Event()" in code_content, "Semantic Error: Should use threading.Event for the async-wait pattern."
    assert ".wait(" in code_content, "Must implement a blocking wait on the synchronization primitive."

# --- 3. ROS2 Lifecycle Management ---
def test_service_readiness(code_content):
    """Ensures the model understands ROS2 service discovery."""
    assert "wait_for_service" in code_content, "Must implement wait_for_service logic."
    assert "service_is_ready" in code_content or "wait_for_service" in code_content, \
        "Failed to verify service availability before request."

# --- 4. Callback Logic ---
def test_future_handling(code_content):
    """Checks if the model knows how to trigger the end of the wait."""
    # It must either use add_done_callback or poll the future (callback is preferred)
    assert "add_done_callback" in code_content or "future.done()" in code_content, \
        "Must implement a mechanism to detect future completion."

# --- 5. Instruction Following: Specific Hints ---
def test_error_specification(code_content):
    """Verifies that the model uses the exact error hints required by the benchmark."""
    # These hints are crucial for the rqt_reconfigure UI to display correct error messages
    assert "timed out waiting for service" in code_content
    assert "the target node may not be spinning" in code_content
    assert "AsyncServiceCallFailed" in code_content

# --- 6. Absence of ROS1 Legacy ---
def test_no_rospy_artifacts(code_content):
    """Ensure the model is not hallucinating ROS1 threading or service calls."""
    assert "rospy" not in code_content
    assert "Condition" not in code_content, "In ROS2 rqt_reconfigure, Event is the standard pattern."
