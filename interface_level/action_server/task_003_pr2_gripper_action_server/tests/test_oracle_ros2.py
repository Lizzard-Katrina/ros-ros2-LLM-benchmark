import re
from pathlib import Path

TRANSLATED_FILE = "joint_trajectory_action.cpp"

def load_code() -> str:
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()

# -------------------------------------------------
# Test Group 0: ROS1 artifacts must be removed
# -------------------------------------------------
def test_no_ros1_artifacts():
    """
    Concept: ROS1 APIs must not appear in the translated ROS2 action server
    """
    code = load_code()
    forbidden = [
        "ros/ros.h",
        "actionlib/server/action_server.h",
        "ros::init",
        "ros::NodeHandle",
        "ros::Publisher",
        "ros::Subscriber"
    ]
    for pat in forbidden:
        assert pat not in code, f"ROS1 artifact found: {pat}"

# -------------------------------------------------
# Test Group 1: ROS2 action server usage
# -------------------------------------------------
def test_ros2_action_server_used():
    """
    Concept: Uses rclcpp_action server (ActionServer or create_server)
    """
    code = load_code()
    assert re.search(r"(rclcpp_action|rclcpp::Node)", code), \
        "ROS2 action server library not used"

def test_action_type_present():
    """
    Concept: JointTrajectoryAction type is preserved
    """
    code = load_code()
    assert "JointTrajectoryAction" in code, "JointTrajectoryAction type missing"

# -------------------------------------------------
# Test Group 2: Node and server initialization
# -------------------------------------------------
def test_node_creation():
    """
    Concept: ROS2 Node must be created
    """
    code = load_code()
    assert re.search(r"(rclcpp::Node|rclcpp::init)", code), "ROS2 Node creation not detected"

def test_action_server_creation():
    """
    Concept: Action server is instantiated and bound to goalCB
    """
    code = load_code()
    assert re.search(r"(rclcpp_action::create_server|ActionServer)", code), \
        "Action server instantiation not detected"

# -------------------------------------------------
# Test Group 3: GoalCB execution logic
# -------------------------------------------------
def test_goalCB_exists():
    """
    Concept: goalCB function exists
    """
    code = load_code()
    assert re.search(r"goalCB\s*\(", code), "goalCB function not found"

def test_goalCB_handles_trajectory():
    """
    Concept: goalCB publishes trajectory and accepts new goals
    """
    code = load_code()
    assert re.search(r"(publish|set_accepted|set_canceled)", code), \
        "goalCB does not handle trajectory/goals correctly"

# -------------------------------------------------
# Test Group 4: ControllerStateCB execution logic
# -------------------------------------------------
def test_controllerStateCB_exists():
    """
    Concept: controllerStateCB function exists
    """
    code = load_code()
    assert re.search(r"controllerStateCB\s*\(", code), "controllerStateCB function not found"

def test_controllerStateCB_feedback_handling():
    """
    Concept: controllerStateCB updates goal state based on feedback
    """
    code = load_code()
    assert re.search(r"(set_succeeded|set_aborted|TrajectoryControllerState)", code), \
        "controllerStateCB feedback handling missing"
