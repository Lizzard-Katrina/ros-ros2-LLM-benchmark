import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "generic_laser_filter_node.cpp"

@pytest.fixture
def code_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        content = f.read()
        return content

# --- 1. FilterChain API Signature Check ---
def test_filter_chain_interface_usage(code_content):
    pattern = r"filter_chain_\.configure\s*\(.*?get_node_logging_interface.*?get_node_parameters_interface"
    assert re.search(pattern, code_content, re.DOTALL), \
        "API Signature Error: logging_interface must be passed before parameters_interface in FilterChain::configure."

# --- 2. TF MessageFilter Configuration ---
def test_tf_filter_binding(code_content):
    assert "tf_filter_.registerCallback" in code_content, "Missing registration of the TF Filter callback."
    tolerance_pattern = r"setTolerance\s*\(\s*(?:\w+::)?(?:milliseconds\s*\(\s*30\s*\)|30ms|0\.03s)\s*\)"
    assert re.search(tolerance_pattern, code_content), "TF Filter tolerance must be set to 30ms using std::chrono."

# --- 3. ROS 2 Communication & QoS ---
def test_qos_and_topics(code_content):
    assert '"output"' in code_content, "Publisher topic 'output' not found."
    assert "SensorDataQoS" in code_content, "Must use SensorDataQoS for laser scan publishers/subscriptions."

# --- 4. Timer-based Deprecation Warning ---
def test_deprecation_timer(code_content):
    timer_pattern = r"create_wall_timer\s*\(\s*(?:\w+::)?(?:seconds\s*\(\s*5\s*\)|5s).*?RCLCPP_WARN"
    assert re.search(timer_pattern, code_content, re.DOTALL), "A 5-second wall timer with a warning log is required."
    assert "scan_to_scan_filter_chain" in code_content, "Warning log must suggest migrating to 'scan_to_scan_filter_chain'."

# --- 5. Style & Architecture Constraints ---
def test_chrono_and_bind_usage(code_content):
    assert "std::bind" in code_content, "Constraint Violation: Must use std::bind for callback registration."
    assert "std::placeholders::_1" in code_content, "Must use std::placeholders::_1 for message filter arguments."

# --- 6. Absence of Legacy ROS 1 Symbols ---
def test_no_ros1_symbols(code_content):
    legacy_patterns = [
        r"ros::NodeHandle",
        r"ros::init",
        r"ros::Subscriber",
        r"ros::Publisher",
        r"ros::ok\(\)",
        r"ros::Rate",
        r"ros::spin",
        r"nh\.subscribe"
    ]
    for pattern in legacy_patterns:
        assert not re.search(pattern, code_content), f"Legacy ROS 1 symbol detected: {pattern}. Migration is incomplete."