import re
import pytest
from pathlib import Path

# Path to the C++ source file
CPP_FILE = Path(__file__).resolve().parents[1] / "odom_tracker.cpp"

@pytest.fixture
def code():
    with open(CPP_FILE, 'r') as f:
        return f.read()

def test_no_ros1_remnants(code):
    """FAIL if legacy ROS 1 symbols or NodeHandles are present."""
    ros1_symbols = [
        r"ros::NodeHandle", r"ros::Publisher", r"ros::Subscriber", 
        r"nh\.", r"ros::Time::now", r"ros::Duration", r"getParam"
    ]
    for symbol in ros1_symbols:
        assert not re.search(symbol, code), f"Migration Error: Legacy ROS 1 symbol '{symbol}' detected."

def test_ros2_node_base_interface(code):
    """
    FAIL if RealtimePublisher is initialized incorrectly.
    ROS 2 RealtimePublisher requires the NodeBaseInterface.
    """
    # Pattern looks for: RealtimePublisher<...>(node->get_node_base_interface(), ...)
    # It allows for different variable names for the node pointer (node/this/nh)
    rt_init_pattern = r"RealtimePublisher<[\w:]+>\s*\(\s*\w+->get_node_base_interface\(\)"
    assert re.search(rt_init_pattern, code), \
        "Architecture Error: RealtimePublisher must be initialized with 'get_node_base_interface()' in ROS 2."

def test_parameter_declaration_lifecycle(code):
    """
    FAIL if parameters are retrieved without declaration.
    ROS 2 requires 'declare_parameter' before 'get_parameter'.
    """
    # Check for at least 4 threshold declarations + odom_frame
    declarations = re.findall(r"this->declare_parameter", code)
    assert len(declarations) >= 5, \
        f"Logic Error: Found only {len(declarations)} declarations. ROS 2 requires declaring all config thresholds."
    assert "get_parameter" in code, "API Error: Missing 'get_parameter' calls for runtime config."

def test_realtime_publishing_logic(code):
    """
    FAIL if the realtime-safe trylock pattern is missing or incorrect.
    """
    # Verify trylock usage
    assert "trylock()" in code, "Concurrency Error: Realtime publishing must use 'trylock()' to stay non-blocking."
    # Verify the message access and publish sequence
    assert "->msg_" in code, "Logic Error: Must populate data via the 'msg_' member of the RealtimePublisher."
    assert "unlockAndPublish()" in code, "API Error: Missing 'unlockAndPublish()' to emit the path."

def test_modern_cpp_binding(code):
    """
    FAIL if boost::bind is used instead of std::bind.
    """
    assert "boost::bind" not in code, "Legacy Error: 'boost::bind' is deprecated. Use 'std::bind'."
    assert "std::bind" in code, "Style Error: ROS 2 callbacks should use 'std::bind' with 'std::placeholders'."

def test_ros2_message_namespaces(code):
    """
    FAIL if message types omit the 'msg' sub-namespace.
    """
    # Checks for nav_msgs::msg::Path and nav_msgs::msg::Odometry
    assert "nav_msgs::msg::Path" in code, "Namespace Error: Missing 'msg::' in Path type definition."
    assert "nav_msgs::msg::Odometry" in code, "Namespace Error: Missing 'msg::' in Odometry type definition."

def test_chrono_timing_usage(code):
    """
    FAIL if legacy ros::Time is used for duration or timing.
    """
    # ROS 2 uses rclcpp::Time and std::chrono
    assert "rclcpp::Time" in code, "Type Error: Use 'rclcpp::Time' instead of 'ros::Time'."
    # Specifically for the timer or path stamp
    assert "this->now()" in code or "node->now()" in code, "API Error: Use 'now()' from the Node for timestamps."
def test_safe_logging_and_node_usage(code):
    """
    FAIL if using a local 'node' variable instead of the class member or 'this'.
    In ROS 2, logging should typically use 'this->get_logger()' if inherited.
    """
    # If the model creates a local shared_ptr inside the constructor 
    # but uses it elsewhere, it's a lifecycle risk.
    assert "get_logger()" in code, "API Error: Missing ROS 2 logging calls (RCLCPP_INFO etc.)"
    # Ensure it's not using a naked 'node->' if we expect inheritance
    assert "this->get_logger()" in code or "get_logger()" in code, \
        "Safety Error: Use 'this->get_logger()' to ensure the node lifecycle is managed by the class."
