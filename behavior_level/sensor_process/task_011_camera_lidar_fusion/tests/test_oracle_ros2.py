import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "imm_ukf_pda.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()

def test_tf2_buffer_pointer_usage():
    """Matches 'tf_buffer_->lookupTransform' specifically as per TODO constraint."""
    content = get_content()
    # Fixed regex to specifically look for pointer access
    pattern = r"tf_buffer_->\s*lookupTransform\s*\("
    assert re.search(pattern, content), "Failure: Should use pointer access 'tf_buffer_->' for lookupTransform."

def test_tf2_time_lookup_accuracy():
    """Concept: Ensure the transform lookup uses the message timestamp, not just zero."""
    content = get_content()
    # Check if header.stamp is passed to lookupTransform
    pattern = r"lookupTransform\s*\([\s\S]+?header\.stamp"
    assert re.search(pattern, content), "Failure: lookupTransform should use 'input.header.stamp' for temporal accuracy."

def test_tf2_geometry_msgs_include():
    """Concept: Ensure necessary conversion headers are included."""
    content = get_content()
    assert "tf2_geometry_msgs/tf2_geometry_msgs.hpp" in content, "Failure: Missing required header for tf2::fromMsg conversion."

def test_namespace_full_compliance():
    """Checks for correct ROS 2 message namespacing."""
    content = get_content()
    # Check both function signature and variable declarations
    pattern = r"autoware_msgs::msg::DetectedObjectArray"
    assert re.search(pattern, content), "Failure: DetectedObjectArray must be in '::msg::' namespace."

def test_ukf_pipeline_integrity():
    """Verifies that the core math logic is not skipped."""
    content = get_content()
    assert ".prediction(" in content, "Failure: Missing prediction call."
    assert "probabilisticDataAssociation(" in content, "Failure: Missing PDA call."
    assert ".update(" in content, "Failure: Missing UKF update call."

def test_no_ros1_symbols():
    """Ensures no ROS 1 legacy API remains."""
    content = get_content()
    bad_symbols = ["ros::Time", "ros::Duration", ".toSec()", "tf::TransformListener"]
    for symbol in bad_symbols:
        assert symbol not in content, f"Failure: Legacy ROS 1 symbol '{symbol}' found."
