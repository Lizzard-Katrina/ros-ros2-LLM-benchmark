import re
from pathlib import Path

# ==========
# Helpers
# ==========

def load_code():
    """
    Load all C++ source files produced by LLM.
    Assumes translated code is placed under src/.
    """
    cpp_files = Path("src").rglob("*.cpp")
    content = ""
    for f in cpp_files:
        content += f.read_text(encoding="utf-8", errors="ignore")
    return content


# ==========
# Tests
# ==========

def test_uses_ros2_headers_not_ros1():
    """
    Concept: ROS2 headers must be used; ROS1 headers must be absent.
    """
    code = load_code()

    assert re.search(r"#include\s*<rclcpp/[^>]+>", code), \
        "ROS2 rclcpp headers not found"

    forbidden = [
        r"#include\s*<ros/ros.h>",
        r"#include\s*<nodelet/nodelet.h>",
        r"PLUGINLIB_EXPORT_CLASS",
    ]

    for pattern in forbidden:
        assert not re.search(pattern, code), \
            f"ROS1 artifact found: {pattern}"


def test_component_or_node_class_exists():
    """
    Concept: Nodelet should be migrated to ROS2 Node or Component.
    """
    code = load_code()

    assert re.search(
        r"class\s+\w+\s*:\s*public\s+rclcpp::(Node|NodeOptions)",
        code
    ), "No ROS2 Node / Component class found"


def test_ros2_parameter_usage():
    """
    Concept: ros::NodeHandle::getParam -> ROS2 parameter API
    """
    code = load_code()

    assert re.search(
        r"declare_parameter\s*<\s*double\s*>|\bdeclare_parameter\s*\(",
        code
    ), "ROS2 declare_parameter not found"

    assert re.search(
        r"get_parameter\s*\(",
        code
    ), "ROS2 get_parameter not found"

    assert not re.search(r"getParam\s*\(", code), \
        "ROS1 getParam should not exist"


def test_publisher_creation_ros2_style():
    """
    Concept: advertise() -> create_publisher()
    """
    code = load_code()

    assert re.search(
        r"create_publisher\s*<\s*std_msgs::msg::\w+\s*>",
        code
    ), "ROS2 create_publisher not found"

    assert not re.search(r"\.advertise\s*\(", code), \
        "ROS1 advertise() detected"


def test_subscription_creation_ros2_style():
    """
    Concept: subscribe() -> create_subscription()
    """
    code = load_code()

    assert re.search(
        r"create_subscription\s*<\s*std_msgs::msg::\w+\s*>",
        code
    ), "ROS2 create_subscription not found"

    assert not re.search(r"\.subscribe\s*\(", code), \
        "ROS1 subscribe() detected"


def test_callback_uses_ros2_message_types():
    """
    Concept: std_msgs::Float64 -> std_msgs::msg::Float64
    """
    code = load_code()

    assert re.search(
        r"std_msgs::msg::(Float64|Bool|Byte|Time)",
        code
    ), "ROS2 message namespace std_msgs::msg::* not found"

    assert not re.search(
        r"std_msgs::(Float64|Bool|Byte|Time)\b",
        code
    ), "ROS1 message type detected (missing ::msg)"


def test_no_ros1_logging_or_shared_ptr():
    """
    Concept: ROS1 logging & boost shared_ptr must be removed.
    """
    code = load_code()

    forbidden = [
        r"ROS_INFO",
        r"ROS_DEBUG",
        r"NODELET_DEBUG",
        r"boost::shared_ptr",
    ]

    for pattern in forbidden:
        assert not re.search(pattern, code), \
            f"Forbidden ROS1 pattern found: {pattern}"


def test_node_spin_and_init_present():
    """
    Concept: rclcpp::init + spin OR component registration
    """
    code = load_code()

    assert (
        re.search(r"rclcpp::init\s*\(", code)
        or re.search(r"RCLCPP_COMPONENTS_REGISTER_NODE", code)
    ), "Neither rclcpp::init nor component registration found"


def test_namespace_semantics_preserved():
    """
    Concept: global / private / namespaced topics should still exist
    (semantic check via string presence)
    """
    code = load_code()

    expected_topics = [
        r'"/global"',
        r'"namespaced"',
        r'"private"',
        r'"in"',
        r'"out"',
    ]

    for topic in expected_topics:
        assert re.search(topic, code), \
            f"Expected topic name not found: {topic}"
