import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "moveit_cpp_tutorial.cpp"


def get_clean_content():
    """Strip comments from the source file to avoid false positives in documentation."""
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Remove single line comments
    content = re.sub(r'//.*', '', content)
    # Remove multi-line comments
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    return content

def test_ros2_node_lifecycle_setup():
    """Concept: Proper ROS 2 Node initialization with MoveIt-compatible options."""
    content = get_clean_content()
    # Check for mandatory NodeOptions to unlock the parameter server
    assert "automatically_declare_parameters_from_overrides" in content, \
        "Failure: NodeOptions must enable parameter overrides for MoveIt 2 YAML loading."
    assert "allow_undeclared_parameters" in content, \
        "Failure: NodeOptions must allow undeclared parameters for dynamic plugin loading."
    # Check if the node is actually instantiated using these options
    assert re.search(r"Node\s*\(.*,\s*.*options\s*\)", content) or \
           re.search(r"make_shared<rclcpp::Node>\s*\(.*,\s*.*options\s*\)", content), \
        "Failure: Node must be instantiated with the configured NodeOptions object."

def test_moveit_cpp_namespace_accuracy():
    """Concept: MoveItCpp resides in moveit_cpp namespace in ROS 2, not planning_interface."""
    content = get_clean_content()
    # Critical migration error: MoveItCpp moved namespaces in ROS 2
    forbidden = "moveit::planning_interface::MoveItCpp"
    assert forbidden not in content, \
        f"Failure: Detected ROS 1 style namespace '{forbidden}'. In ROS 2, use 'moveit_cpp::MoveItCpp'."
    assert "moveit_cpp::MoveItCpp" in content, \
        "Failure: The correct namespace 'moveit_cpp::MoveItCpp' was not found."

def test_async_execution_paradigm():
    """Concept: Non-blocking execution to prevent MoveIt initialization deadlocks."""
    content = get_clean_content()
    # MoveItCpp requires a background spinner to process internal state updates
    has_async = any(x in content for x in ["std::thread", "MultiThreadedExecutor"])
    assert has_async, "Failure: MoveItCpp requires a background thread or MultiThreadedExecutor."
    assert "spin" in content, "Failure: No spin loop detected for the background executor."

def test_planning_scene_availability():
    """Concept: Explicitly providing the planning scene service in ROS 2."""
    content = get_clean_content()
    # This is no longer automatic in MoveItCpp; it must be called to be visible to RViz
    assert "providePlanningSceneService" in content, \
        "Failure: Missing call to 'providePlanningSceneService()'."

def test_clean_migration_no_ros1_symbols():
    """Concept: Total removal of legacy ROS 1 API symbols from functional code."""
    content = get_clean_content()
    # These should be replaced by rclcpp equivalents
    ros1_symbols = ["ros::init", "ros::NodeHandle", "ros::AsyncSpinner", "ros::Duration", "ros::ok"]
    for symbol in ros1_symbols:
        assert symbol not in content, f"Failure: Legacy ROS 1 symbol '{symbol}' found in executable code."

def test_message_namespace_migration():
    """Concept: ROS 2 uses nested namespaces (::msg::) for messages."""
    content = get_clean_content()
    # Standard ROS 2 message path check: geometry_msgs::msg::PoseStamped
    assert "geometry_msgs::msg::" in content, \
        "Failure: Message namespaces must include '::msg::' (e.g., geometry_msgs::msg::PoseStamped)."
    assert "PoseStamped" in content
