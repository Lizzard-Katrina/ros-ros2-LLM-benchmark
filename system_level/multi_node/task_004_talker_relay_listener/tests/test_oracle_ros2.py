import re
import pytest
from pathlib import Path

# Paths to the generated files
BASE_DIR = Path(__file__).resolve().parents[1]
SERVER_FILE = BASE_DIR / "add_two_inits_server.cpp"
CLIENT_FILE = BASE_DIR / "add_two_inits_client.cpp"
BABBLER_FILE = BASE_DIR / "babbler.cpp"

def read_file(path):
    return path.read_text() if path.exists() else ""

# --- SERVER TESTS ---

def test_server_callback_signature():
    """Verify ROS2 shared_ptr signature for service callback."""
    content = read_file(SERVER_FILE)
    pattern = r"std::shared_ptr\s*<\s*.*TwoInts::(?:Request|Response)\s*>"
    assert re.search(pattern, content), "Server callback must use ROS2 shared_ptr signatures."

def test_server_creation_logic():
    """Verify service server creation and naming."""
    content = read_file(SERVER_FILE)
    assert "create_service" in content, "Failed to find 'create_service' call."
    assert '"add_two_ints"' in content or "'add_two_ints'" in content, "Service name mismatch."

# --- CLIENT TESTS ---

def test_client_wait_for_service():
    content = read_file(CLIENT_FILE)
    pattern = r"wait_for_service\s*\(\s*[\s\S]+?\s*\)"
    assert re.search(pattern, content), "Client must wait for service with a timeout/condition."

def test_client_async_request():
    """Verify non-blocking service request pattern."""
    content = read_file(CLIENT_FILE)
    assert "async_send_request" in content, "Client must use asynchronous requests to avoid blocking the executor."

def test_client_result_handling():
    """Verify handling of futures or results from the service."""
    content = read_file(CLIENT_FILE)
    # Look for .get() on a future or wait_for calls
    assert any(x in content for x in [".get()", "wait_for", "spin_until_future_complete"]), \
        "Client must implement logic to retrieve the result from the async future."

# --- BABBLER TESTS ---

def test_babbler_timer_paradigm():
    """Verify paradigm shift from while-loop to Timer."""
    content = read_file(BABBLER_FILE)
    assert "create_wall_timer" in content, "Babbler must use a Timer for periodic publishing in ROS2."
    assert "while" not in content or "rclcpp::ok" in content, "Legacy while-loop detected in periodic task."

def test_babbler_executor_spin():
    """Verify use of executor to handle async callbacks."""
    content = read_file(BABBLER_FILE)
    assert "spin" in content, "Node must use an executor (spin) to process timer callbacks."

# --- NEGATIVE / CLEANLINESS TESTS ---

def test_absence_of_legacy_artifacts():
    """Strictly verify that no ROS 1 artifacts remain."""
    for file in [SERVER_FILE, CLIENT_FILE, BABBLER_FILE]:
        content = read_file(file)
        assert "ros::NodeHandle" not in content, f"Legacy NodeHandle found in {file.name}."
        assert "ros::init" not in content, f"Legacy ros::init found in {file.name}."
        assert "ros::Publisher" not in content, f"Legacy ros::Publisher found in {file.name}."
        assert not re.search(r"\bROS_(?:INFO|ERROR|WARN)\b", content), f"Legacy ROS_INFO macros found in {file.name}."
