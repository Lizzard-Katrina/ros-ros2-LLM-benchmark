import re
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
CLIENT_FILE = BASE_DIR / "mp3_controller.py"


def read_file(path: Path) -> str:
    assert path.exists(), f"Expected file {path} to exist"
    return path.read_text(encoding="utf-8")


# =========================
# Structural / API-level
# =========================

def test_ros2_client_uses_node():
    """
    Client should define a ROS2 Node
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"class\s+\w+\s*\(\s*Node\s*\)",
        code
    ), "ROS2 client must define a Node subclass"


def test_ros2_client_creates_service_client():
    """
    Client must create a service client for MP3InventoryService
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"create_client\s*\(\s*MP3InventoryService\s*,",
        code
    ), "Client must create a service client using MP3InventoryService"


def test_ros2_client_waits_for_service():
    """
    Client should wait for service availability
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"wait_for_service\s*\(",
        code
    ), "Client must wait for the service using wait_for_service(...)"


# =========================
# Interface semantics
# =========================

def test_client_populates_request_fields():
    """
    Client must populate request fields defined in the .srv file
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"request\.\w+",
        code
    ), "Client must set fields on the service request (e.g., request.request_string)"


def test_client_accesses_response_list():
    """
    Client must access the list_strings field in the service response
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"response\.\s*list_strings",
        code
    ), "Client must access response.list_strings from MP3InventoryService"


def test_client_uses_response_to_drive_loop():
    """
    Client should iterate over service response data (albums / titles)
    """
    code = read_file(CLIENT_FILE)
    assert re.search(
        r"for\s+\w+\s+in\s+response\.list_strings",
        code
    ), "Client should iterate over response.list_strings"


# =========================
# Negative / cleanup tests
# =========================

def test_no_ros1_api_usage():
    """
    Translated ROS2 code must not contain rospy or ServiceProxy
    """
    code = read_file(CLIENT_FILE)
    assert "rospy." not in code, "ROS2 client must not use rospy"
    assert "ServiceProxy" not in code, "ROS2 client must not use ROS1 ServiceProxy"

