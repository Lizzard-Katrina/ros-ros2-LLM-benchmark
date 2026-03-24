import re
from pathlib import Path

TRANSLATED_FILE = "turtlebot3_patrol_server.py"

def load_code() -> str:
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()


# -------------------------------------------------
# Test Group 0: ROS1 artifacts must be removed
# -------------------------------------------------

def test_no_ros1_artifacts():
    """
    Concept: Ensure ROS1 APIs are not present in the translated ROS2 code
    """
    code = load_code()
    forbidden = [
        "rospy",
        "actionlib",
        "SimpleActionServer",
        "rospy.init_node",
        "rospy.sleep",
    ]
    for pat in forbidden:
        assert pat not in code, f"ROS1 artifact found: {pat}"


# -------------------------------------------------
# Test Group 1: ROS2 action server usage
# -------------------------------------------------

def test_ros2_action_server_used():
    """
    Concept: ROS2 ActionServer library is used
    """
    code = load_code()
    assert re.search(r"(rclcpp_action|rclpy\.action)", code), \
        "ROS2 action server library not detected"


def test_action_type_present():
    """
    Concept: Patrol action type is preserved in ROS2 code
    """
    code = load_code()
    assert re.search(r"Patrol", code), "Patrol action type missing"


# -------------------------------------------------
# Test Group 2: Node and server initialization
# -------------------------------------------------

def test_node_creation():
    """
    Concept: ROS2 node is created correctly
    """
    code = load_code()
    assert re.search(r"(Node\s*\(|create_node)", code), "ROS2 Node creation not detected"


def test_action_server_creation():
    """
    Concept: Action server is instantiated and bound to execute callback
    """
    code = load_code()
    patterns = [
        r"ActionServer",
        r"create_server",
    ]
    for pat in patterns:
        assert re.search(pat, code), f"Action server instantiation not detected: {pat}"


# -------------------------------------------------
# Test Group 3: Execute callback semantics
# -------------------------------------------------

def test_execute_callback_exists():
    """
    Concept: execute_callback function exists and is defined
    """
    code = load_code()
    assert re.search(r"def\s+execute_callback\s*\(", code), "execute_callback not defined"

def test_execute_callback_contains_patrol_logic():
    """
    Concept: execute_callback references the patrol goal and decides pattern
    """
    code = load_code()
    # Check for goal references
    assert re.search(r"goal_msg\.goal\.x", code), "Patrol goal type not referenced in execute_callback"
    assert re.search(r"goal_handle", code), "Goal handle usage missing in execute_callback"
    # Check for patrol method calls
    assert re.search(r"square\s*\(", code) or re.search(r"triangle\s*\(", code), \
        "Patrol shape functions not called in execute_callback"
