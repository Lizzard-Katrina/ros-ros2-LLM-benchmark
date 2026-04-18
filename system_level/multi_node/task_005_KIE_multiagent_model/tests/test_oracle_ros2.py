import re
import pytest
from pathlib import Path

# ================= Configuration =================
BASE_DIR = Path(__file__).resolve().parents[1] 
MASTER_DISCOVERY_FILE = BASE_DIR / "master_discovery.py"
INTERFACE_FINDER_FILE = BASE_DIR / "interface_finder.py"

# ================= Fixtures =================
@pytest.fixture
def master_discovery_code():
    if not MASTER_DISCOVERY_FILE.exists():
        pytest.skip(f"File not found: {MASTER_DISCOVERY_FILE}")
    return MASTER_DISCOVERY_FILE.read_text(encoding='utf-8')

@pytest.fixture
def interface_finder_code():
    if not INTERFACE_FINDER_FILE.exists():
        pytest.skip(f"File not found: {INTERFACE_FINDER_FILE}")
    return INTERFACE_FINDER_FILE.read_text(encoding='utf-8')

# ================= Task 1: master_discovery.py Tests =================

def test_md_ros2_publisher_definition(master_discovery_code):
    assert "create_publisher" in master_discovery_code
    assert re.search(r"self\.pubchanges\s*=\s*.*\.create_publisher\(\s*MasterState", master_discovery_code)
    assert re.search(r"self\.pubstats\s*=\s*.*\.create_publisher\(\s*LinkStatesStamped", master_discovery_code)

def test_md_no_rospy_time_leakage(master_discovery_code):
    assert "rospy.Time" not in master_discovery_code

def test_md_udp_protocol_preservation(master_discovery_code):
    assert "struct.pack" in master_discovery_code
    assert "Discoverer.HEARTBEAT_FMT" in master_discovery_code

def test_md_variable_consistency(master_discovery_code):
    assert re.search(r"self\.pubchanges\s*=", master_discovery_code)
    assert re.search(r"self\.pubchanges\.publish\(", master_discovery_code)

def test_md_proper_msg_import(master_discovery_code):
    assert "fkie_mas_msgs.msg" in master_discovery_code
    assert "from rclpy" in master_discovery_code

# ================= Task 2: interface_finder.py Tests =================

def test_if_no_xmlrpc_usage(interface_finder_code):
    assert "xmlrpc" not in interface_finder_code
    assert "ServerProxy" not in interface_finder_code

def test_if_graph_api_usage(interface_finder_code):
    assert "get_topic_names_and_types" in interface_finder_code

def test_if_host_filtering_logic(interface_finder_code):
    assert "get_hostname" in interface_finder_code
    assert "own_host" in interface_finder_code

def test_if_wait_loop_mechanism(interface_finder_code):
    assert "while" in interface_finder_code
    assert "time.sleep" in interface_finder_code

def test_if_return_type_contract(interface_finder_code):
    assert re.search(r"result\s*=\s*\[\]", interface_finder_code)
    assert re.search(r"return\s+result", interface_finder_code)

def test_if_no_rospy_in_finder(interface_finder_code):
    assert "import rospy" not in interface_finder_code
    assert "rospy.is_shutdown()" not in interface_finder_code
