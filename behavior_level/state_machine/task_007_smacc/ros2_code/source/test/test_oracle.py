import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "smacc_client_async_behavior.cpp"

@pytest.fixture
def code():
    with open(CPP_FILE, 'r') as f:
        return f.read()

def test_api_migration_cleanliness(code):
    """Detect ROS 1 remnants and ensure ROS 2 logging macros are used."""
    ros1_remnants = [r"ROS_", r"ros::ok", r"ros::Rate", r"ros::spin"]
    for remnant in ros1_remnants:
        assert not re.search(remnant, code), f"Migration Error: Detected ROS 1 legacy '{remnant}'."
    assert "RCLCPP_" in code and "getLogger()" in code, "API Error: Missing ROS 2 logging (RCLCPP_ macros)."

def test_executor_and_future_logic(code):
    """Check for executor-safe spinning and non-blocking future polling."""
    assert "ros::spinOnce" not in code, "Deadlock Risk: Manual spinOnce detected."
    assert "wait_for" in code and "std::future_status" in code, \
        "Logic Error: Must use wait_for with future_status to poll the thread safely."

def test_functional_preservation(code):
    """Ensure the user-defined onExit() is still triggered asynchronously."""
    assert "onExit()" in code, "Functional Error: onExit() logic was lost during migration."
    assert "onExitThread_" in code and "std::async" in code, \
        "Functional Error: Missing asynchronous launch of onExitThread_."

def test_memory_safety_and_types(code):
    """Check for lambda safety and correct ROS 2 types."""
    if "[=]" in code and "this->" in code:
        assert any(x in code for x in ["shared_from_this", "weak_ptr", "std::bind"]), \
            "Safety Warning: Risky [=] capture of 'this' without lifetime protection."
    assert "rclcpp::Rate" in code, "Type Error: Must use rclcpp::Rate."

def test_lifecycle_on_entry_init(code):
    """Verify onEntry initiates the thread and posts finish event."""
    assert "onEntryThread_" in code and "postFinishEventFn_" in code, \
        "Logic Error: onEntry must start thread and ensure finish event is posted."