import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "HectorMappingRos.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()

# --- Section 1: Style Compliance (Fixing False Positives) ---

def test_tf2_lookup_syntax():
    """Concept: Ensure pointer-style access to tf_buffer_ for consistency."""
    content = get_content()
    assert "tf_buffer_->lookupTransform" in content

def test_inverse_transform_usage():
    """Concept: Ensure map->odom is calculated using the .inverse() method."""
    content = get_content()
    assert "inverse()" in content or "inverse (" in content

# --- Section 2: New Leak Detections (QoS & Time) ---

def test_qos_durability_policy():
    """Concept: Verify that TransientLocal QoS is used for persistent data."""
    content = get_content()
    qos_pattern = r"(?:TransientLocal|transient_local)"
    assert re.search(qos_pattern, content, re.IGNORECASE), "Failure: Missing TransientLocal QoS durability."

def test_standard_tf2_time_api():
    """Concept: Verify standard tf2_ros conversion instead of manual chrono hacks."""
    content = get_content()
    assert "tf2_ros::fromMsg" in content
    # Block manual nanosecond addition which is error-prone in ROS 2
    assert "chrono::nanoseconds" not in content, "Failure: Use tf2_ros::fromMsg for timestamps."

# --- Section 3: Core Logic & ROS 2 Standards ---

def test_ros2_logging_and_time():
    """Concept: Uses ROS 2 specific logging and timing APIs."""
    content = get_content()
    # Check for rclcpp logging and time source
    assert "RCLCPP_" in content, "Failure: Should use RCLCPP_ logging macros."
    assert "this->get_logger()" in content, "Failure: Missing get_logger() call."
    assert "this->now()" in content or "now()" in content, "Failure: Should use ROS 2 clock/now()."

def test_timestamp_preservation():
    """Concept: Ensure sensor timestamp sync across the pipeline."""
    content = get_content()
    # Matches scan->header.stamp (pointer) or scan.header.stamp (ref)
    pattern = r"header\.stamp\s*=\s*scan[-.]>header\.stamp"
    assert re.search(pattern, content)

def test_zero_ros1_leakage():
    """Concept: Ensure no legacy ROS 1 symbols remain."""
    content = get_content()
    legacy_symbols = ["ros::Time", "tf::Transform", "ros::NodeHandle", "ros::ok()"]
    for sym in legacy_symbols:
        assert sym not in content, f"Detected legacy symbol: {sym}"
