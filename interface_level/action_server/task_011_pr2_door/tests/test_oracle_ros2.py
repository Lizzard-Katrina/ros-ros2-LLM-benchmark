import re
from pathlib import Path
import pytest

# 指向被测代码文件
PY_FILE = Path(__file__).resolve().parents[1] / "door_demo_test_exec_test.py"

@pytest.fixture
def code():
    return PY_FILE.read_text()

# --------------------------
# 语义与语法检查 Testcases
# --------------------------

def test_ros2_node_initialized(code):
    """
    Check that the code initializes a ROS2 node with rclpy
    Semantic: ROS2 node exists
    """
    pattern = r"rclpy\.init\s*\(.*\)"
    assert re.search(pattern, code), "ROS2 node initialization (rclpy.init) missing"

def test_door_action_client_created(code):
    """
    Check that an ActionClient for Door action is created
    Semantic: Door action client exists
    """
    pattern = r"ActionClient\s*\(\s*.*['\"]move_through_door['\"].*,\s*Door\s*\)"
    assert re.search(pattern, code), "Door ActionClient creation missing"

def test_move_base_action_client_created(code):
    """
    Check that an ActionClient for MoveBase action is created
    Semantic: MoveBase action client exists
    """
    pattern = r"ActionClient\s*\(\s*.*['\"]move_base_local['\"].*,\s*MoveBase\s*\)"
    assert re.search(pattern, code), "MoveBase ActionClient creation missing"

def test_wait_for_server_called(code):
    """
    Check that the code waits for the action servers
    Semantic: waits for server
    """
    pattern = r"\.wait_for_server\s*\(\s*\)"
    matches = re.findall(pattern, code)
    assert len(matches) >= 2, "wait_for_server() not called for both action clients"

def test_send_goal_and_wait_called(code):
    """
    Check that the code sends goal and waits for result
    Semantic: goal sent to action client
    """
    pattern = r"\.send_goal.*\(.*\)"
    matches = re.findall(pattern, code)
    assert len(matches) >= 2, "send_goal or wait not called for both action clients"

def test_subscriber_exists(code):
    """
    Check that a ROS2 subscriber exists for test_output
    Semantic: subscriber for logging or test output exists
    """
    pattern = r"Subscriber\s*\(\s*['\"]\/test_output['\"]"
    assert re.search(pattern, code), "Subscriber to /test_output missing"

def test_goal_initialization(code):
    """
    Check that DoorGoal and MoveBaseGoal are defined
    Semantic: goal messages initialized
    """
    door_pattern = r"DoorGoal\s*\(\s*\)"
    move_pattern = r"MoveBaseGoal\s*\(\s*\)"
    assert re.search(door_pattern, code), "DoorGoal not initialized"
    assert re.search(move_pattern, code), "MoveBaseGoal not initialized"
