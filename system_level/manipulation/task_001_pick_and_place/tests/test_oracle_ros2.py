import re
from pathlib import Path

# File paths using the requested format
DETECTOR_FILE = Path(__file__).resolve().parents[1] / "object_detector.py"
SM_FILE = Path(__file__).resolve().parents[1] / "pick_and_place_state_machine.py"

def get_content(file_path):
    with open(file_path, "r") as f:
        return f.read()

def test_detector_async_non_blocking_pattern():
    """
    Verifies that the detector does not use ROS 1 blocking service patterns.
    Ensures the use of ROS 2 Futures or Async calls to prevent Executor stalls.
    """
    content = get_content(DETECTOR_FILE)
    
    # Check for call_async usage instead of blocking ServiceProxy
    assert re.search(r"\.call_async\s*\(", content), \
        "System Level Failure: Detector uses blocking service calls. Must use 'call_async' for ROS 2 compatibility."
    
    # Ensure no ROS 1 service artifacts remain
    assert not re.search(r"ServiceProxy|wait_for_service", content), \
        "Anti-Leakage Failure: ROS 1 ServiceProxy or blocking wait detected in detector."

def test_cross_component_lifecycle_orchestration():
    content = get_content(SM_FILE)
    
    assert re.search(r"(MultiThreadedExecutor|SingleThreadedExecutor|executor\.add_node)", content), \
        "Architecture Failure: Multi-node systems must use an Executor."
    assert re.search(r"(rclpy\.spin|executor\.spin)", content), \
        "Lifecycle Failure: Missing system-level spin logic (expected rclpy.spin or executor.spin)."
def test_state_machine_future_synchronization():
    """
    Verifies the handshake between the State Machine and the Controller.
    Transitions must wait for the asynchronous ROS 2 service/action response.
    """
    content = get_content(SM_FILE)
    
    # Check for Future handling logic (callbacks or spin_until_future_complete)
    # This ensures the 'select_object' -> 'pick_object' transition is synchronized.
    sync_pattern = r"(\.add_done_callback|spin_until_future_complete|\.done\(\))"
    assert re.search(sync_pattern, content), \
        "Logic Consistency Failure: State machine transitions occur without waiting for the Controller's async response."

def test_static_interface_linkage_consistency():
    """
    Verifies that both files have synchronized their message definitions and topic naming.
    """
    det_content = get_content(DETECTOR_FILE)
    sm_content = get_content(SM_FILE)
    
    # Check for ROS 2 message import synchronization
    import_pattern = r"from\s+pick_and_place\.msg\s+import\s+(?:DetectedObjectsStamped|DetectedObject)"
    assert re.search(import_pattern, det_content), "Interface Failure: Detector has incorrect ROS 2 msg import path."
    
    # Ensure topic naming consistency across the system boundary
    assert "/object_detection" in det_content and "/object_detection" in sm_content, \
        "Linkage Failure: Topic name mismatch between Detector (Publisher) and State Machine system (Subscriber)."

def test_parameter_declaration_compliance():
    """
    Verifies adherence to the ROS 2 parameter declaration style.
    Static consistency check for the node configuration interface.
    """
    content = get_content(DETECTOR_FILE)
    
    # ROS 2 requires explicit declaration of parameters in the node
    assert re.search(r"self\.declare_parameter\s*\(", content), \
        "Style Restriction Failure: Node parameters must be explicitly declared using 'self.declare_parameter'."

def test_clean_migration_anti_leakage():
    """
    Final check to ensure no ROS 1 artifacts remain that would cause system-level runtime crashes.
    """
    combined_content = get_content(DETECTOR_FILE) + get_content(SM_FILE)
    
    # List of strictly forbidden ROS 1 symbols
    forbidden = [r"rospy\.", r"roslib\.", r"queue_size\s*=", r"anonymous=True"]
    
    for pattern in forbidden:
        assert not re.search(pattern, combined_content), \
            f"Migration Integrity Failure: Residual ROS 1 symbol '{pattern}' detected in system files."
