import re
import pytest
from pathlib import Path

NODE_FILE = Path(__file__).resolve().parents[1] / "serial_node.py"
CLIENT_FILE = Path(__file__).resolve().parents[1] / "SerialClient.py"

def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# --- Tests for serial_node.py (The Wrapper) ---

def test_node_inheritance():
    """Check if SerialNode correctly inherits from rclpy.node.Node."""
    content = get_content(NODE_FILE)
    # Match: class Name(Node): or class Name(rclpy.node.Node):
    pattern = r"class\s+\w+\s*\((?:rclpy\.node\.)?Node\)\s*:"
    assert re.search(pattern, content), \
        "Failure: SerialNode must be a class inheriting from rclpy.node.Node."

def test_parameter_declaration_pattern():
    """Verify ROS 2 parameter declaration and retrieval style."""
    content = get_content(NODE_FILE)
    # Check for .declare_parameter(...) and .get_parameter(...).value
    decl_pattern = r"\.declare_parameter\s*\("
    get_pattern = r"\.get_parameter\s*\(.*\)\.(?:get_parameter_value\(\)\.)?value"
    assert re.search(decl_pattern, content) and re.search(get_pattern, content), \
        "Failure: Must use declare_parameter() and get_parameter().value for configuration."

def test_dependency_injection_linkage():
    """Verify that the node instance (self) is passed to SerialClient."""
    content = get_content(NODE_FILE)
    # Matches SerialClient(..., self, ...) or SerialClient(self, ...)
    pattern = r"SerialClient\s*\(.*self.*\)"
    assert re.search(pattern, content), \
        "Failure: The Node instance (self) must be passed to the SerialClient constructor (Linkage Error)."

def test_executor_spin():
    """Ensure the node uses rclpy.spin() for lifecycle management."""
    content = get_content(NODE_FILE)
    assert re.search(r"rclpy\.spin\s*\(", content), \
        "Failure: Missing rclpy.spin() to keep the node and its callbacks alive."

# --- Tests for SerialClient.py (The Engine) ---

def test_client_constructor_interface():
    """Verify SerialClient constructor is updated to accept the injected node."""
    content = get_content(CLIENT_FILE)
    # Matches def __init__(self, node, ...): or def __init__(self, port, baud, node):
    pattern = r"def\s+__init__\s*\(self\s*,\s*.*node"
    assert re.search(pattern, content), \
        "Failure: SerialClient.__init__ must be refactored to accept a node instance."

def test_node_clock_usage():
    """Verify migration from global rospy.Time to Node-based Clock API."""
    content = get_content(CLIENT_FILE)
    # Match node.get_clock().now() or self.node.get_clock().now()
    pattern = r"\w+\.get_clock\s*\(\s*\)\.now\s*\(\s*\)"
    assert re.search(pattern, content), \
        "Failure: Time operations must use the injected node's clock (node.get_clock().now())."

def test_node_logger_usage():
    """Verify migration from rospy.log* to Node-based Logger API."""
    content = get_content(CLIENT_FILE)
    # Match node.get_logger().info/warn/error
    pattern = r"\w+\.get_logger\s*\(\s*\)\.(?:info|warn|error|fatal|debug)"
    assert re.search(pattern, content), \
        "Failure: Logging must use the injected node's logger (node.get_logger())."

def test_communication_interface_creation():
    """Verify publishers/subscribers are created via the Node API."""
    content = get_content(CLIENT_FILE)
    # Check for node.create_publisher(Type, 'topic', ...)
    pattern = r"\.create_publisher\s*\(\s*[\w\.]+\s*,\s*['\"]"
    assert re.search(pattern, content), \
        "Failure: ROS 2 interfaces must be created via node.create_publisher() or node.create_subscription()."

# --- Cross-cutting Tests (Safety & Anti-Leakage) ---

def test_anti_leakage_rospy():
    """Ensure no ROS 1 (rospy) artifacts remain in either file."""
    node_content = get_content(NODE_FILE)
    client_content = get_content(CLIENT_FILE)
    combined = node_content + client_content
    
    # Check for 'import rospy' or any 'rospy.xxx' calls
    assert not re.search(r"\bimport\s+rospy\b", combined), "Failure: 'import rospy' found. ROS 1 artifacts must be removed."
    assert not re.search(r"rospy\.", combined), "Failure: 'rospy' calls found. All logic must use rclpy."

def test_absence_of_global_node_init():
    """Ensure rclpy.create_node is not called inside the Engine (should be injected)."""
    content = get_content(CLIENT_FILE)
    assert not re.search(r"rclpy\.create_node", content), \
        "Failure: SerialClient should use the injected node, not create its own global node."
