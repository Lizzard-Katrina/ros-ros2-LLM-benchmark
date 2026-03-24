import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "move_base_state.py"

@pytest.fixture
def code():
    with open(PY_FILE, 'r') as f:
        return f.read()

# 1. FIXED: Anti-ROS 1 Remnants (Excluding Class Names)
def test_no_ros1_remnants(code):
    """Ensure no legacy actionlib terms exist in ACTIVE logic."""
    forbidden_terms = [
        "actionlib", 
        "actionlib_msgs", 
        "GoalStatus.SUCCEEDED", # Old ROS 1 constant
        "GoalStatus.ABORTED"
    ]
    for term in forbidden_terms:
        # Regex: matches the term if NOT preceded by 'class ' and NOT in a comment
        pattern = rf"^(?!.*#)(?!.*class ).*{re.escape(term)}"
        assert not re.search(pattern, code, re.MULTILINE), \
            f"Legacy Error: Found ROS 1 remnant '{term}' in active logic."

# 2. FIXED: Nav2 Topic with Strict Formatting
def test_nav2_topic_naming(code):
    """Enforce the exact topic name as specified in the style guide."""
    assert "'/navigate_to_pose'" in code or '"/navigate_to_pose"' in code, \
        "Style Error: Use the absolute topic name '/navigate_to_pose' as requested."

# 3. CRITICAL: Detection of Proxy Misuse (The "Silent Killer")
def test_flexbe_proxy_compliance(code):
    """
    Detects if the LLM is bypassing FlexBE's ProxyActionClient 
    by trying to manage ROS 2 futures manually.
    """
    # Managing futures manually in FlexBE states often leads to NoneType errors
    forbidden_methods = ["get_result_async", "_result_future", "future.result"]
    for method in forbidden_methods:
        assert method not in code, \
            f"Architecture Error: Do not manage futures manually. Use ProxyActionClient.has_result()."
    
    # Ensure the developer uses the provided Proxy polling method
    assert "self._client.has_result" in code, \
        "API Error: Must use self._client.has_result(topic) for non-blocking polling."

# 4. Logic Integrity: ROS 2 GoalStatus
def test_ros2_goal_status_constants(code):
    """Verify usage of ROS 2 specific GoalStatus names."""
    assert "STATUS_SUCCEEDED" in code, \
        "Type Error: Use 'GoalStatus.STATUS_SUCCEEDED' for ROS 2 action status."

# 5. Clean Environment
def test_placeholder_removal(code):
    """Ensure the 'NotImplementedError' is completely purged."""
    assert "NotImplementedError" not in code, "Logic Error: NotImplementedError was not removed."
