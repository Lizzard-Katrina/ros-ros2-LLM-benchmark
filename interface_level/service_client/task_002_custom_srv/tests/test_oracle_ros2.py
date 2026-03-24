import re
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
CLIENT_FILE = BASE_DIR / "ros_client.py"
SERVER_FILE = BASE_DIR / "ros_server.py"


def read_file(path: Path) -> str:
    assert path.exists(), f"Expected file not found: {path.name}"
    return path.read_text(encoding="utf-8")




# =========================
# Server-side tests
# =========================

def test_ros2_server_uses_node_subclass():
    """
    Server should define a ROS2 Node (class inheriting from Node)
    """
    code = read_file(SERVER_FILE)
    assert re.search(
        r"class\s+\w+\s*\(\s*Node\s*\)",
        code
    ), "ROS2 server should define a class inheriting from rclpy.node.Node"


def test_ros2_server_creates_custom_service():
    """
    Server should create a custom service using create_service with AddThreeInts
    """
    code = read_file(SERVER_FILE)
    assert re.search(
        r"create_service\s*\(\s*AddThreeInts\s*,",
        code
    ), "Server must create a service with AddThreeInts using create_service(...)"



def test_server_defines_handler_and_accesses_request():
    """
    Semantic core:
    Server must define a handler and access request fields (req.a / req.b / req.c)
    """
    code = read_file(SERVER_FILE)
    assert re.search(
        r"def\s+handle_add_three_ints\s*\(",
        code
    ), "Server must define handle_add_three_ints(...)"
    assert re.search(
        r"req\.\w+",
        code
    ), "Handler must access request fields (e.g., req.a, req.b, req.c)"


def test_ros2_server_spins():
    """
    Server should spin the node (lifecycle correctness)
    """
    code = read_file(SERVER_FILE)
    assert re.search(
        r"rclpy\.spin\s*\(",
        code
    ), "Server should spin the ROS2 node using rclpy.spin(...)"



# =========================
# Client-side tests
# =========================

def test_ros2_client_creates_client_with_correct_type():
    """
    Client should create a service client using AddThreeInts
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"create_client\s*\(\s*AddThreeInts\s*,",
        code
    ), "Client must create a service client with AddThreeInts"


def test_ros2_client_waits_for_service():
    """
    Client should wait for service availability
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"wait_for_service\s*\(",
        code
    ), "Client must wait for the service using wait_for_service(...)"



def test_ros2_client_calls_service():
    """
    Client should call the service (async or sync call is acceptable)
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"call_async\s*\(|call\s*\(",
        code
    ), "Client must call the service (call_async(...) or call(...))"
