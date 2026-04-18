import re
import pytest
from pathlib import Path

# Paths to the migrated files
RECORDER_CPP = Path(__file__).resolve().parents[1] / "recorder.cpp"
TALKER_PY = Path(__file__).resolve().parents[1] /"talker.py"

def load_file(path):
    if not path.exists():
        return ""
    return path.read_text()

# --- Recorder (C++ Logic Engine) Tests ---

def test_recorder_generic_intent():
    """Verify if the model understands Generic Subscription (Type Agnostic)."""
    content = load_file(RECORDER_CPP)
    # Check for keywords indicating a generic/serialized subscription approach
    patterns = [r"generic_subscription", r"ShapeShifter", r"SerializedMessage"]
    assert any(re.search(p, content, re.IGNORECASE) for p in patterns), \
        "Fail: Recorder should use Generic Subscription or ShapeShifter for universal recording."

def test_recorder_qos_awareness():
    """Verify handling of Distributed Compatibility (QoS)."""
    content = load_file(RECORDER_CPP)
    # Match any logic indicating BestEffort or SensorData QoS
    qos_patterns = r"(SensorDataQoS|best_effort|BEST_EFFORT|ReliabilityPolicy::BestEffort)"
    assert re.search(qos_patterns, content, re.IGNORECASE), \
        "Fail: Missing QoS compatibility logic (BestEffort/SensorData) for sensor support."

def test_recorder_clock_source():
    """Verify time source alignment with ROS Domain."""
    content = load_file(RECORDER_CPP)
    # Must call a node-based 'now' and exclude standard C++ chrono for recording timestamps
    assert re.search(r"now\(\)", content), "Fail: Must use node->now() for timestamps."
    assert "std::chrono::system_clock" not in content, "Fail: Should not use system clock in ROS 2."

# --- Talker: Intent Validation ---

def test_talker_parameter_intent():
    """Verify implementation of Dynamic Topic Configuration."""
    content = load_file(TALKER_PY)
    # Ensure parameter declaration and retrieval are present
    assert "declare_parameter" in content, "Fail: Node must declare parameters for dynamic config."
    assert "get_parameter" in content, "Fail: Node must retrieve parameters for the topic name."

def test_talker_timer_intent():
    """Verify non-blocking execution model (Timer)."""
    content = load_file(TALKER_PY)
    # Check for Timer logic and correct frequency (10Hz)
    assert "create_timer" in content, "Fail: Should use a ROS 2 Timer instead of while-loops."
    assert re.search(r"(0\.1|1/10|1\.0/10)", content), "Fail: Timer frequency should be 10Hz (0.1s)."

def test_talker_clock_usage():
    """Verify message payload contains ROS-synchronized time."""
    content = load_file(TALKER_PY)
    # Check for node-based clock access
    assert re.search(r"get_clock\(.*?\)\.now\(.*?\)", content), \
        "Fail: Talker message payload must use ROS clock for system-level sync."

# --- Cross-Node: Clean Migration Check ---

def test_absence_of_legacy_ros1():
    content_py = load_file(TALKER_PY)
    content_cpp = load_file(RECORDER_CPP)
    # Use word boundaries \b to ensure we match 'import rospy' but not 'test_rospy'
    ros1_artifacts = [r"\brospy\b", r"\bros::init\b", r"\bros::NodeHandle\b"]

    for art_pattern in ros1_artifacts:
        assert not re.search(art_pattern, content_py), f"Legacy artifact '{art_pattern}' detected."
        assert not re.search(art_pattern, content_cpp), f"Legacy artifact '{art_pattern}' detected."
