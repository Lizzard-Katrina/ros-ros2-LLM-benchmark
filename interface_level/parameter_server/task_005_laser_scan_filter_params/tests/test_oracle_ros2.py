import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "generic_laser_filter_node.cpp"

@pytest.fixture
def code_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        content = f.read()
        return content
        #return re.sub(r'//.*|/\*.*?\*/', '', content, flags=re.DOTALL)

# --- 1. FilterChain API Signature Check ---
def test_filter_chain_interface_usage(code_content):
    """
    Verifies that FilterChain::configure follows the correct ROS 2 signature.
    Correct Order: (prefix, logging_interface, parameters_interface).
    This catches models that swap parameter/logging interfaces, which causes compilation errors.
    """
    # Pattern looks for logging interface appearing before parameters interface inside configure()
    pattern = r"filter_chain_\.configure\s*\(.*?get_node_logging_interface.*?get_node_parameters_interface"
    assert re.search(pattern, code_content, re.DOTALL), \
        "API Signature Error: logging_interface must be passed before parameters_interface in FilterChain::configure."

# --- 2. TF MessageFilter Configuration ---
def test_tf_filter_binding(code_content):
    """
    Verifies that the TF MessageFilter is correctly bound to the callback and has 
    the required 30ms synchronization tolerance.
    """
    # Check for callback registration
    assert "tf_filter_.registerCallback" in code_content, "Missing registration of the TF Filter callback."
    
    # Check for 30ms tolerance using chrono-style literals or duration objects
    tolerance_pattern = r"setTolerance\s*\(\s*(?:\w+::)?(?:milliseconds\s*\(\s*30\s*\)|30ms|0\.03s)\s*\)"
    assert re.search(tolerance_pattern, code_content), "TF Filter tolerance must be set to 30ms using std::chrono."

# --- 3. ROS 2 Communication & QoS ---
def test_qos_and_topics(code_content):
    """
    Validates that the 'output' publisher is created and that SensorDataQoS is 
    utilized for high-frequency laser scan data as requested in the TODO.
    """
    # Ensure the 'output' topic is present
    assert '"output"' in code_content, "Publisher topic 'output' not found."
    
    # Ensure SensorDataQoS is used for sensor streams
    assert "SensorDataQoS" in code_content, "Must use SensorDataQoS for laser scan publishers/subscriptions."

# --- 4. Timer-based Deprecation Warning ---
def test_deprecation_timer(code_content):
    """
    Checks for a 5-second recurring timer and the presence of the 
    specific deprecation warning message.
    """
    # Match the 5s wall timer, allowing for multi-line formatting
    timer_pattern = r"create_wall_timer\s*\(\s*(?:\w+::)?(?:seconds\s*\(\s*5\s*\)|5s).*?RCLCPP_WARN"
    assert re.search(timer_pattern, code_content, re.DOTALL), "A 5-second wall timer with a warning log is required."
    
    # Ensure the model advises switching to the correct new node
    assert "scan_to_scan_filter_chain" in code_content, "Warning log must suggest migrating to 'scan_to_scan_filter_chain'."

# --- 5. Style & Architecture Constraints ---
def test_chrono_and_bind_usage(code_content):
    """
    Strictly enforces the use of modern C++ features (std::bind, placeholders) 
    as specified in the task constraints.
    """
    assert "std::bind" in code_content, "Constraint Violation: Must use std::bind for callback registration."
    assert "std::placeholders::_1" in code_content, "Must use std::placeholders::_1 for message filter arguments."

# --- 6. Absence of Legacy ROS 1 Symbols ---
def test_no_ros1_symbols(code_content):
    """
    Ensures that no legacy ROS 1 symbols or patterns remain in the code.
    Matches common ROS 1 patterns like NodeHandle, ros::init, and ros::Subscriber.
    """
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
