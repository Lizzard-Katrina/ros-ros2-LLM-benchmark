import re
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
CLIENT_FILE = BASE_DIR / "ros_client.py"
SERVER_FILE = BASE_DIR / "ros_server.py"


def read_file(path: Path) -> str:
    assert path.exists(), f"Expected file not found: {path.name}"
    return path.read_text(encoding="utf-8")




# ============================================================
# CLIENT — ROS2 CORE SEMANTICS
# ============================================================

def test_client_uses_rclpy_and_node():
    """
    Client should use ROS2 initialization and node abstraction
    """
    code = read_file(CLIENT_FILE)
    assert re.search(r"rclpy\.init\s*\(", code), \
        "ROS2 client must initialize rclpy using rclpy.init()"
    assert re.search(r"Node\s*\(", code), \
        "ROS2 client must create or use a Node abstraction"


def test_client_creates_service_client():
    """
    Client should create a ROS2 service client
    """
    code = read_file(CLIENT_FILE)
    assert re.search(r"create_client\s*\(", code), \
        "ROS2 client must create a service client using create_client(...)"


def test_client_waits_for_service():
    """
    Client should wait for the service to become available
    """
    code = read_file(CLIENT_FILE)
    assert re.search(r"wait_for_service\s*\(", code), \
        "ROS2 client must wait for the service to become available"


def test_client_calls_service_async():
    """
    Client should call the service asynchronously
    """
    code = read_file(CLIENT_FILE)
    assert re.search(r"call_async\s*\(", code), \
        "ROS2 client must call the service using call_async(...)"


# ============================================================
# CLIENT — NO ROS1 ARTIFACTS
# ============================================================

def test_client_has_no_ros1_artifacts():
    """
    Client must not contain ROS1-specific APIs
    """
    code = read_file(CLIENT_FILE)
    assert not re.search(r"\brospy\b", code), \
        "ROS2 client must not contain rospy"
    assert not re.search(r"ServiceProxy", code), \
        "ROS2 client must not use rospy.ServiceProxy"


# ============================================================
# SERVER — ROS2 CORE SEMANTICS
# ============================================================

def test_server_creates_service():
    """
    Server should create a ROS2 service
    """
    code = read_file(SERVER_FILE)
    assert re.search(r"create_service\s*\(", code), \
        "ROS2 server must create a service using create_service(...)"


def test_server_spins():
    """
    Server should spin to process incoming requests
    """
    code = read_file(SERVER_FILE)
    assert re.search(r"rclpy\.spin\s*\(", code), \
        "ROS2 server must spin to handle service requests"


# ============================================================
# SERVER — NO ROS1 ARTIFACTS
# ============================================================

def test_server_has_no_ros1_artifacts():
    """
    Server must not contain ROS1-specific APIs
    """
    code = read_file(SERVER_FILE)
    assert not re.search(r"\brospy\b", code), \
        "ROS2 server must not contain rospy"
