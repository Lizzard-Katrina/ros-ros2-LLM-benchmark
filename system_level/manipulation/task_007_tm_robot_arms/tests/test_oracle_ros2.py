import re
from pathlib import Path
import pytest

# Paths to the three modified files
SERVICE_CPP = Path(__file__).resolve().parents[1] / "tm_ros_service.cpp"
COMM_CPP = Path(__file__).resolve().parents[1] / "tm_communication.cpp"
DEMO_PY = Path(__file__).resolve().parents[1] / "ask_item_demo.py"

def get_content(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

# --- 1. Thread Synchronization (tm_ros_service.cpp) ---
def test_sync_wait_logic():
    """Concept: Service must block using a condition variable until the hardware responds."""
    content = get_content(SERVICE_CPP)
    # Check for the presence of a wait_for on the condition variable
    pattern = r"svr_cond_\.wait_for\s*\(\s*\w+,\s*(?:std|boost)::chrono::duration"
    assert re.search(pattern, content), \
        "Missing synchronous wait logic. The service must wait for the SVR response thread."

def test_mutex_safety():
    """Concept: Access to shared state (svr_updated_) must be protected by a mutex."""
    content = get_content(SERVICE_CPP)
    # Check for unique_lock or lock_guard wrapping the shared state
    pattern = r"(?:unique_lock|lock_guard).*?svr_mtx_"
    assert re.search(pattern, content), \
        "Thread safety violation. Shared state access must be protected by 'svr_mtx_'."

# --- 2. Low-Level Communication (tm_communication.cpp) ---
def test_socket_polling():
    """Concept: Use non-blocking I/O (select/poll) to prevent the driver from hanging."""
    content = get_content(COMM_CPP)
    # Ensure select() is used with a timeout
    assert re.search(r"select\s*\(.*?&tv\)", content), \
        "Hardware communication must use 'select()' with a timeout to avoid blocking the executor."

def test_recv_error_handling():
    """Concept: Properly handle TCP socket closure (recv returning 0)."""
    content = get_content(COMM_CPP)
    # Search for handling of 0 return value from recv
    pattern = r"recv\(.*?\)\s*==\s*0"
    assert re.search(pattern, content), \
        "Incomplete socket logic: Must handle the case where 'recv' returns 0 (connection closed)."

# --- 3. Application Logic & Parsing (ask_item_demo.py) ---
def test_tm_protocol_parsing():
    """Concept: Parse the specific TM string format 'Item={val1,val2...}'."""
    content = get_content(DEMO_PY)
    # Look for string splitting or regex extraction of the curly brace content
    pattern = r"(?:\.split\(|[rR]?['\"].*?\{.*?\}['\"])"
    assert re.search(pattern, content), \
        "The demo script must implement parsing logic to extract values from TM's '{...}' string format."

def test_demo_blocking_call():
    """Concept: Correct usage of the 'wait_time' parameter in the service call."""
    content = get_content(DEMO_PY)
    # Ensure ask_item is called with a wait_time > 0 for blocking requests
    pattern = r"ask_item\(.*?,.*?,[^0]\d*\)" 
    assert re.search(pattern, content), \
        "Demo failed to implement a blocking service call (wait_time must be > 0)."

# --- 4. Manipulation Mapping (tm_ros_service.cpp) ---
def test_motion_type_coverage():
    """Concept: Map ROS service request types to specific TM motion commands."""
    content = get_content(SERVICE_CPP)
    # Check if all three primary motion interfaces are called
    interfaces = ["set_joint_pos_PTP", "set_tool_pose_PTP", "set_tool_pose_Line"]
    for interface in interfaces:
        assert interface in content, f"Missing mapping for motion command: {interface}"

# --- 5. Anti-Pattern Check ---
def test_no_legacy_ros1_symbols():
    """Concept: Ensure no ROS 1 symbols remained during the migration."""
    content = get_content(SERVICE_CPP)
    # ROS 2 uses shared pointers and different node handles
    legacy_symbols = [r"ros::ok\(", r"ros::init\(", r"ros::NodeHandle"]
    for sym in legacy_symbols:
        assert not re.search(sym, content), f"Legacy ROS 1 symbol detected: {sym}"

def test_svr_callback_notification():
    """Concept: The SVR callback must notify the waiting service thread."""
    content = get_content(SERVICE_CPP)
    assert re.search(r"svr_cond_\.notify_(?:all|one)\s*\(", content), \
        "Logic Fail: Missing condition variable notification. The service will always timeout."

def test_python_brace_stripping():
    """Concept: Must handle TM Robot's brace format specifically."""
    content = get_content(DEMO_PY)
    assert re.search(r"\.strip\s*\(\s*['\"].*?[{}]", content) or re.search(r"replace\s*\(.*?[{}]", content), \
        "Protocol Fail: The demo does not strip the '{' or '}' characters from the response string."
