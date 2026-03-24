import re
from pathlib import Path


TRANSLATED_FILE = "calibration_action_server_stub.py"


def load_code() -> str:
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()


# -------------------------------------------------
# Test Group 0: ROS1 artifacts must be removed
# -------------------------------------------------

def test_no_ros1_artifacts():
    """
    Concept: ROS1 APIs must not appear in ROS2 action server
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
    Concept: Uses rclcpp_action server (ActionServer or create_server)
    """
    code = load_code()
    assert re.search(
        r"(rclcpp_action|rclpy\.action)",
        code
    ), "ROS2 action server library not used"

def test_action_type_present():
    """
    Concept: RobotCalibration action type is preserved
    """
    code = load_code()
    assert re.search(r"RobotCalibration", code), \
        "RobotCalibration action type missing"


# -------------------------------------------------
# Test Group 2: Node and server initialization
# -------------------------------------------------

def test_node_creation():
    """
    Concept: ROS2 node is created
    """
    code = load_code()
    assert re.search(r"(create_node|Node\s*\()", code), \
        "ROS2 Node creation not detected"


def test_action_server_creation():
    """
    Concept: Action server is instantiated and bound to execute callback
    """
    code = load_code()
    patterns = [
        r"create_server",
        r"ActionServer",
    ]
    assert re.search(
        r"create_server\s*\(|ActionServer\s*\(",
        code
    ), "Action server not instantiated"

# -------------------------------------------------
# Test Group 3: Execute callback semantics
# -------------------------------------------------

def test_execute_callback_exists():
    """
    Concept: Execute callback function exists
    """
    code = load_code()
    assert re.search(
        r"(execute|handle_accepted)\s*\(.*\)",
        code
    ), "Execute callback function not defined"


def test_feedback_and_result_used():
    """
    Concept: Feedback and Result messages are used
    """
    code = load_code()
    assert re.search(
        r"publish_feedback",
        code
    ), "Feedback not published"

    assert re.search(
        r"(set_succeeded|succeed|abort)",
        code
    ), "Result not reported"


# -------------------------------------------------
# Test Group 4: Progress loop + preemption semantics
# -------------------------------------------------

def test_progress_loop_present():
    """
    Concept: Iterative progress loop (stress / long-running action)
    """
    code = load_code()
    assert re.search(r"for\s*\(|while\s*\(", code), \
        "Progress loop not found"


def test_success_or_abort_reported():
    """
    Concept: Action result is set (succeed / abort / cancel)
    """
    code = load_code()
    patterns = [
        r"succeed",
        r"abort",
        r"set_succeeded",
        r"setAborted",
    ]
    assert any(re.search(p, code) for p in patterns), \
        "Action result completion not detected"
