import re
import pytest
from pathlib import Path

# Path to the migrated file
PYTHON_FILE = Path(__file__).resolve().parents[1] / "pick_and_place.py"

@pytest.fixture
def source_code():
    if not PYTHON_FILE.exists():
        pytest.fail(f"File not found: {PYTHON_FILE}")
    with open(PYTHON_FILE, 'r') as f:
        return f.read()

## --- 1. Architectural Consistency: Shared Node Instance ---

def test_shared_node_instance(source_code):
    """Verify that a single rclpy Node is created and shared across all interfaces."""
    # Capture the variable name assigned to the rclpy node
    node_match = re.search(r'(\w+)\s*=\s*rclpy\.create_node\(', source_code)
    assert node_match, "System Error: No rclpy.create_node() found. ROS 2 requires an explicit Node handle."
    
    node_var = node_match.group(1)
    
    # Check if all three interfaces reference the same node variable
    interfaces = [
        rf'InterbotixManipulatorXS\(.*node\s*=\s*{node_var}',
        rf'InterbotixPointCloudInterface\(.*node\s*=\s*{node_var}',
        rf'InterbotixArmTagInterface\(.*node\s*=\s*{node_var}'
    ]
    
    for pattern in interfaces:
        assert re.search(pattern, source_code, re.DOTALL), \
            f"Architecture Failure: Interface does not share the node instance '{node_var}'. This causes TF buffer collisions."

## --- 2. Static Consistency: TF Naming Convention ---

def test_tf_naming_convention(source_code):
    """Verify ROS 2 TF frame naming (no leading slashes)."""
    # ROS 2 forbids frames starting with '/'
    illegal_slash = re.search(r'ref_frame\s*=\s*["\']/', source_code)
    assert not illegal_slash, "Static Consistency Failure: Found ROS 1 style frame naming (leading slash). Use 'wx200/base_link' instead."
    
    valid_frame = re.search(r'ref_frame\s*=\s*["\']wx200/base_link["\']', source_code)
    assert valid_frame, "Semantic Failure: The reference frame 'wx200/base_link' is missing or renamed incorrectly."

## --- 3. API Correctness: Constructor Params ---

def test_constructor_params_migration(source_code):
    """Verify that hardware-specific parameters are maintained during migration."""
    assert re.search(r'InterbotixManipulatorXS\(.*robot_model\s*=\s*["\']wx200["\']', source_code), \
        "API Failure: robot_model parameter is missing or incorrectly mapped in the constructor."

## --- 4. Anti-Leakage: ROS 1 Artifacts ---

def test_no_rospy_artifacts(source_code):
    """Ensure no legacy rospy code remains in the system."""
    forbidden_patterns = [
        r'import\s+rospy',
        r'rospy\.init_node',
        r'rospy\.get_param',
        r'rospy\.Rate',
        r'roslaunch' # Found in comments/strings usually, should be ros2 launch
    ]
    for pattern in forbidden_patterns:
        assert not re.search(pattern, source_code), f"Leakage detected: Legacy ROS 1 pattern found -> {pattern}"

## --- 5. System Lifecycle Management ---

def test_system_lifecycle_calls(source_code):
    """Verify presence of rclpy initialization and shutdown."""
    assert re.search(r'rclpy\.init\(', source_code), "Lifecycle Error: rclpy.init() is missing."
    assert re.search(r'rclpy\.shutdown\(', source_code) or re.search(r'node\.destroy_node\(', source_code), \
        "Lifecycle Error: Proper system shutdown or node destruction logic is missing."

## --- 6. Semantic Consistency: Perception Data Handling ---

def test_cluster_position_destructuring(source_code):
    """Verify that perception data access matches the ROS 2 interface structure."""
    # Expected: x, y, z = cluster["position"]
    pattern = r'(\w+),\s*(\w+),\s*(\w+)\s*=\s*cluster\[["\']position["\']\]'
    assert re.search(pattern, source_code), \
        "Logic Failure: Cluster position destructuring (x, y, z = cluster['position']) is missing or malformed."

## --- 7. Dependency Alignment: Imports ---

def test_ros2_package_imports(source_code):
    """Check for correct ROS 2 module imports."""
    assert re.search(r'import\s+rclpy', source_code), "Import Failure: rclpy module not imported."
    # Ensure time-sensitive logic uses ROS 2 compatible duration if time.sleep is replaced
    # This is a soft check for modern rclpy patterns
    assert not re.search(r'from\s+std_msgs\.msg\s+import', source_code) or \
           re.search(r'from\s+std_msgs\.msg\s+import\s+Header', source_code), \
           "Consistency Warning: Check if message imports follow the 'package.msg' format."
