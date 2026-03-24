import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "scenario_state_base.py"

@pytest.fixture
def code():
    with open(PY_FILE, 'r') as f:
        return f.read()

# 1. Check Node Injection and Variable Name (Strict Style)
def test_node_init_style(code):
    """Verify 'node' is accepted without type hints and stored as 'self.node'."""
    # Matches def __init__(self, node, ...):  BUT fails on node: Node
    init_pattern = r"def\s+__init__\s*\(\s*self\s*,\s*node\s*,"
    storage_pattern = r"self\.node\s*=\s*node"
    assert re.search(init_pattern, code), "Style Violation: Constructor must use 'node' without type hints."
    assert re.search(storage_pattern, code), "Logic Violation: Node handle must be stored in 'self.node'."

# 2. Check Parameter Logic (Conceptual requirement)
def test_parameter_logic(code):
    """Ensure parameters are declared and the '.value' attribute is accessed."""
    assert re.search(r"self\.node\.declare_parameter", code), "Missing parameter declaration logic."
    assert r".value" in code, "ROS 2 parameters require accessing the '.value' attribute."

# 3. Check Topic Integrity (Deep logic check)
def test_topic_and_msg_integrity(code):
    """Verify that the subscription hasn't been accidentally changed to 'ActionDispatch'."""
    # Original logic was ActionFeedback on /kcl_rosplan/action_feedback
    # This catches the LLM 'hallucinating' a different topic
    pattern = r"create_subscription\s*\(.*ActionFeedback.*['\"]/kcl_rosplan/action_feedback['\"]"
    assert re.search(pattern, code), "Topic mismatch: Subscription must use ActionFeedback on the feedback topic."

# 4. Check QoS Durability (Behavioral requirement)
def test_qos_latching(code):
    """Check for TRANSIENT_LOCAL durability to match ROS 1 latching."""
    assert re.search(r"TRANSIENT_LOCAL", code, re.IGNORECASE), "Latching behavior (TransientLocal QoS) is missing."

# 5. Check Interface Parameter Passing (Logic consistency)
def test_interface_params_retention(code):
    """Ensure the ontology interface still receives url and prefix strings, not just the node."""
    # This checks if the LLM preserved the original data flow
    pattern = r"DomesticOntologyInterface\s*\(\s*[^,]+url[^,]*,\s*[^,]+prefix"
    assert re.search(pattern, code), "Data Flow Error: Ontology interface must receive the url and prefix parameters."

# 6. Check Clock API
def test_clock_api(code):
    """Verify ROS 2 node clock usage."""
    assert "rospy.Time" not in code, "Legacy rospy.Time detected."
    assert "self.node.get_clock().now()" in code.replace(" ", ""), "Incorrect ROS 2 clock access."

# 7. Check Legacy Symbols (Cleanliness)
def test_no_rospy_remnants(code):
    assert "import rospy" not in code, "rospy import remains."
    assert "latch=True" not in code, "ROS 1 latching syntax used in ROS 2."
