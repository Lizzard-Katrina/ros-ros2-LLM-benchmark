import re
import pytest
from pathlib import Path
# Path to the hollowed/truncated file
CPP_FILE = Path(__file__).resolve().parents[1]/ "mapOptmization.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()


# 1. Critical Logic: Timestamp Synchronization
def test_tf_timestamp_synchronization():
    """
    [CRITICAL] Ensure TF is synced with Laser Stamp, NOT current Node time.
    SLAM requires the transform to be valid for the data's time.
    """
    content = get_content()
    # Find the line assigning stamp to the TF message
    stamp_assignment = re.search(r"\.header\.stamp\s*=\s*([^;]+);", content)
    if stamp_assignment:
        val = stamp_assignment.group(1)
        # It must use the laser info stamp, not this->now()
        assert "timeLaserInfoStamp" in val or "timeLaserInfo" in val, \
            "Error: TF stamp must sync with laser data timestamp to avoid drift!"
        assert "this->now()" not in val and "get_clock()->now()" not in val, \
            "Error: Do not use current time for SLAM TF broadcasts."

def test_callback_group_initialization():
    """
    [REAL-TIME] Verify if Callback Groups are mentioned in the constructor.
    LIO-SAM needs this to prevent map optimization from blocking sensor callbacks.
    """
    content = get_content()
    # Check if the code attempts to use Callback Groups
    assert "create_callback_group" in content or "callback_group_" in content, \
        "Error: LIO-SAM migration should use Callback Groups for concurrent execution."
# 3. Concurrency: Data Protection
def test_mutex_locking_in_service():
    """The global map access in service must be protected by a lock."""
    content = get_content()
    # Search for lock_guard or unique_lock in the service function context
    # Usually it locks 'mtx' or similar
    lock_pattern = r"std::lock_guard<std::mutex>\s+\w+\(mtx\)|std::lock_guard"
    assert re.search(lock_pattern, content), \
        "Missing thread-safety (std::lock_guard) in saveMapService while accessing shared clouds."

# 4. ROS 2 API Standard: Service Parameter Types
def test_service_parameter_shared_ptrs():
    """Service parameters must use SharedPtr for Request and Response."""
    content = get_content()
    pattern = r"const\s+std::shared_ptr<[^>]+::Request>\s+\w+,\s*std::shared_ptr<[^>]+::Response>\s+\w+"
    assert re.search(pattern, content), \
        "Service callback parameters must use std::shared_ptr for Request/Response."

# 5. Logic: Service Response Population
def test_service_response_success_set():
    """The success field in response must be explicitly set."""
    content = get_content()
    assert re.search(r"res->success\s*=\s*(?:true|false)", content), \
        "The response field 'res->success' must be populated in the service callback."


# 8. Clean Migration: No ROS 1 Symbols
def test_no_legacy_ros1_symbols():
    """Strictly prohibit any leftover ROS 1 symbols in the migrated code."""
    content = get_content()
    legacy_symbols = ["ros::Time", "ros::ok()", "ROS_INFO", "ros::Publisher", "ros::Subscriber", "tf::"]
    for symbol in legacy_symbols:
        assert symbol not in content, f"Legacy ROS 1 symbol '{symbol}' detected!"
