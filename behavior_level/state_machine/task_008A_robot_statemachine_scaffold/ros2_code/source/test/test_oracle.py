import re
import pytest
from pathlib import Path

# Path to the C++ source file
CPP_FILE = Path(__file__).resolve().parents[1] / "src" / "RobotControlMux.cpp"

@pytest.fixture
def code():
    with open(CPP_FILE, 'r') as f:
        return f.read()

def test_no_ros1_remnants(code):
    """FAIL if legacy ROS 1 symbols exist in code or comments."""
    ros1_symbols = [
        r"ros::NodeHandle", r"ros::Publisher", r"ros::Subscriber", 
        r"ros::ServiceServer", r"privateNh", r"advertiseService",
        r"ros::Duration", r"ros::Timer", r"ros::spin"
    ]
    for symbol in ros1_symbols:
        assert not re.search(symbol, code), f"Migration Error: Legacy ROS 1 symbol '{symbol}' detected."

def test_parameter_lifecycle_strict(code):
    """FAIL if parameters are not declared at least 3 times (topics + timeouts)."""
    declarations = re.findall(r"this->declare_parameter", code)
    assert len(declarations) >= 3, \
        f"Logic Error: Found only {len(declarations)} declarations. ROS 2 requires declaring every parameter."
    assert "get_parameter" in code, "API Error: Missing get_parameter."

def test_ros2_service_signature_naming(code):
    """FAIL if service parameters are renamed to 'req' or 'res' (Violates STYLE guide)."""
    assert "std::shared_ptr" in code, "Type Error: Service callbacks must use std::shared_ptr."
    assert "request->" in code, "Style Error: Mandatory variable 'request->' not found."
    assert "response->" in code, "Style Error: Mandatory variable 'response->' not found."
    assert not re.search(r"Request\s*&\s*", code), "API Error: ROS 1 style '&' reference found in service."

def test_ros2_message_namespaces(code):
    """FAIL if the 'msg' or 'srv' sub-namespace is missing from type names."""
    assert "msg::Twist" in code, "Namespace Error: geometry_msgs::msg::Twist required."
    assert "msg::OperationMode" in code, "Namespace Error: rsm_msgs::msg::OperationMode required."
    assert "srv::SetOperationMode" in code, "Namespace Error: rsm_msgs::srv::SetOperationMode required."

def test_ros2_qos_instantiation(code):
    """FAIL if QoS is used as a simple integer. MUST instantiate a QoS object."""
    qos_obj_pattern = r"rclcpp::QoS\s+[a-zA-Z_][a-zA-Z0-9_]*\s*\("
    qos_inline_pattern = r"rclcpp::QoS\s*\("
    assert re.search(qos_obj_pattern, code) or re.search(qos_inline_pattern, code), \
        "Quality Error: Must explicitly instantiate an rclcpp::QoS object for control topics."

def test_timer_chrono_strict(code):
    """FAIL if timer doesn't use rclcpp and std::chrono."""
    assert "create_wall_timer" in code, "API Error: Missing create_wall_timer."
    assert "std::chrono::" in code, "Type Error: ROS 2 timers require std::chrono (e.g., std::chrono::milliseconds)."
    assert "ros::Duration" not in code, "Legacy Error: Found ros::Duration inside timer."