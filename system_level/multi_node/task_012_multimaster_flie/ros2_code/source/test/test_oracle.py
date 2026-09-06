import re
from pathlib import Path
import pytest

# Adjust paths to find the source files within the package
MONITOR_FILE = Path(__file__).resolve().parents[1] / "task_012_multimaster_flie" / "master_monitor.py"
SYNC_FILE = Path(__file__).resolve().parents[1] / "task_012_multimaster_flie" / "sync_thread.py"

def get_content(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

# --- MasterMonitor Tests ---

def test_monitor_calls_master_api():
    """Verify MasterMonitor retrieves both topic types and system state from the Master API."""
    content = get_content(MONITOR_FILE)
    topic_api = re.search(r'\.getTopicTypes\s*\(', content)
    state_api = re.search(r'\.getSystemState\s*\(', content)
    
    assert topic_api and state_api, (
        "MasterMonitor must call both 'getTopicTypes' and 'getSystemState' "
        "to accurately map the ROS topology."
    )

def test_monitor_uses_succeed_helper():
    """Verify MasterMonitor uses the _succeed helper to validate XML-RPC responses."""
    content = get_content(MONITOR_FILE)
    succeed_pattern = re.search(r'self\._succeed\s*\(', content)
    
    assert succeed_pattern, (
        "MasterMonitor should use the 'self._succeed' helper method to check "
        "XML-RPC return codes (standard ROS Master API practice)."
    )

def test_monitor_populates_master_info():
    """Verify logic exists to populate the MasterInfo object (internal state mapping)."""
    content = get_content(MONITOR_FILE)
    mapping_logic = re.search(r'for\s+\w+,\s+\w+\s+in\s+publishers:', content)
    
    assert mapping_logic, (
        "MasterMonitor must iterate through the extracted system state to "
        "populate internal node-to-topic mapping structures."
    )

# --- SyncThread Tests ---

def test_sync_uses_multicall():
    """Verify SyncThread uses XML-RPC MultiCall for efficient registration."""
    content = get_content(SYNC_FILE)
    multicall_pattern = re.search(r'own_master_multi\s*\(', content)
    
    assert multicall_pattern, (
        "SyncThread must use 'own_master_multi()' (XML-RPC MultiCall) to batch "
        "registration requests, ensuring system-level performance."
    )

def test_sync_loop_prevention():
    """Verify SyncThread checks if the node name matches the local node to prevent infinite loops."""
    content = get_content(SYNC_FILE)
    loop_check = re.search(r'if\s+[\w\.]+\s*==\s*(?:rospy\.get_name\(\)|self\.ros_node_name)', content)
    
    assert loop_check, (
        "SyncThread must implement loop prevention by checking if the remote node "
        "name matches the local sync node's name before registration."
    )

def test_sync_filter_application():
    """Verify the implementation applies filtering rules before synchronizing."""
    content = get_content(SYNC_FILE)
    filter_pattern = re.search(r'\.is_ignored_(?:publisher|subscriber|service)', content)
    
    assert filter_pattern, (
        "SyncThread must call filtering methods (e.g., is_ignored_publisher) "
        "to respect the synchronization policy."
    )

def test_sync_preserves_remote_uri():
    """Verify that the sync logic registers topics using the original remote URI."""
    content = get_content(SYNC_FILE)
    uri_logic = re.search(r'registerPublisher\s*\(.*,\s*[\w_]*uri', content)
    
    assert uri_logic, (
        "SyncThread must use the original remote Node URI when registering "
        "shadow topics to ensure peer-to-peer ROS communication."
    )

def test_absence_of_hardcoded_names():
    """Ensure the code does not use hardcoded local node names."""
    content_monitor = get_content(MONITOR_FILE)
    content_sync = get_content(SYNC_FILE)
    
    hardcoded = re.search(r'[\'"]/(?:master_discovery|master_sync)[\'"]', content_monitor + content_sync)
    
    assert not hardcoded, (
        "The migrated code should use dynamic node name retrieval "
        "(e.g., rospy.get_name()) rather than hardcoded strings."
    )