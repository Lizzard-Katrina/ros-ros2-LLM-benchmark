import re
from pathlib import Path

TRANSLATED_FILE = "head_action_server.py"


def load_code():
    path = Path(__file__).parent.parent / TRANSLATED_FILE
    if not path.exists():
        print("Translated ROS2 file not found")

    code = path.read_text()

    # remove markdown fences
    code = re.sub(r"```[\s\S]*?```", "", code)

    # remove comments
    code = re.sub(r"#.*", "", code)


    return code


# =================================================
# Test Group 0: ROS1 artifacts must be fully removed
# =================================================

def test_no_ros1_artifacts():
    """
    Concept: No ROS1 APIs or concepts remain
    """
    code = load_code()
    forbidden = [
        "rospy",
        "actionlib",
        "SimpleActionServer",
        "rospy.spin",
        "rospy.sleep",
        "rospy.loginfo",
    ]
    for pat in forbidden:
        assert pat not in code, f"ROS1 artifact found: {pat}"


# =================================================
# Test Group 1: ROS2 ActionServer construction
# =================================================

def test_action_server_created():
    """
    Concept: A ROS2 ActionServer must be instantiated
    """
    code = load_code()
    assert re.search(
        r"ActionServer\s*\(",
        code
    ), "ROS2 ActionServer not instantiated"


def test_execute_callback_signature():
    """
    Concept: execute callback must accept goal_handle (ROS2 semantics)
    """
    code = load_code()
    assert re.search(
        r"def\s+execute_callback\s*\(\s*self\s*,\s*goal_handle\s*\)",
        code
    ), "execute_callback must accept goal_handle"


# =================================================
# Test Group 2: Execute callback semantics (CRITICAL)
# =================================================

def test_execute_callback_no_return():
    """
    Concept: Action execution must not behave like a service via return values
    """
    code = load_code()

    forbidden_returns = [
        r"return\s+True",
        r"return\s+False",
        r"return\s+goal",
        r"return\s+result",
    ]

    for pat in forbidden_returns:
        assert not re.search(pat, code), \
            f"Service-style return detected in action server: {pat}"


# =================================================
# Test Group 3: Goal handling semantics
# =================================================

def test_goal_data_accessed():
    """
    Concept: Goal data must be accessed via goal_handle.request
    """
    code = load_code()
    assert re.search(
        r"goal_handle\.request\.\w+",
        code
    ), "Goal data not accessed via goal_handle.request"


# =================================================
# Test Group 4: Long-running execution semantics
# =================================================

def test_long_running_execution_present():
    """
    Concept: Action execution should be long-running
    """
    code = load_code()
    patterns = [
        r"for\s+",
        r"while\s+",
        r"time\.sleep\(",
        r"Rate\(",
    ]
    assert any(re.search(p, code) for p in patterns), \
        "No indication of long-running execution"


# =================================================
# Test Group 5: Feedback semantics
# =================================================

def test_feedback_object_created():
    """
    Concept: Feedback message must be constructed
    """
    code = load_code()
    assert re.search(
        r"Feedback\(",
        code
    ), "Feedback message never constructed"


def test_feedback_published_iteratively():
    """
    Concept: Feedback must be published multiple times
    """
    code = load_code()
    assert len(re.findall(
        r"goal_handle\.publish_feedback",
        code
    )) >= 2, "Feedback not published iteratively"


def test_feedback_progress_updated():
    """
    Concept: Feedback content must change over time
    """
    code = load_code()
    assert re.search(
        r"feedback\.\w+\s*=",
        code
    ), "Feedback fields never updated"


# =================================================
# Test Group 6: Result + goal state lifecycle
# =================================================

def test_result_object_created():
    """
    Concept: Result message must be constructed
    """
    code = load_code()
    assert re.search(
        r"Result\(",
        code
    ), "Result message not created"


def test_terminal_goal_state_set():
    """
    Concept: Action must explicitly set a terminal goal state
    """
    code = load_code()
    patterns = [
        r"goal_handle\.succeed\s*\(",
        r"goal_handle\.abort\s*\(",
        r"goal_handle\.canceled\s*\(",
    ]
    assert any(re.search(p, code) for p in patterns), \
        "No terminal goal state transition detected"


def test_result_set_before_goal_termination():
    """
    Concept: Result must be populated BEFORE succeed/abort/cancel
    """
    code = load_code()

    result_assign = re.search(
        r"result\.\w+\s*=",
        code
    )
    terminal_call = re.search(
        r"goal_handle\.(succeed|abort|canceled)",
        code
    )

    assert result_assign and terminal_call, \
        "Missing result assignment or goal termination"

    assert result_assign.start() < terminal_call.start(), \
        "Result set after goal termination"
