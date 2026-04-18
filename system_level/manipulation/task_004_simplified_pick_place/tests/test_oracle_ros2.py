import re
from pathlib import Path
import pytest

# File path configuration
BASE_PATH = Path(__file__).resolve().parents[1]
FILE_MAIN = BASE_PATH / "ez_pnp2.py"
FILE_TOOLS = BASE_PATH / "ez_tools.py"
FILE_TEST = BASE_PATH / "test2_ez_pnp2.py"

def read_file(path):
    return Path(path).read_text()

## --- 1. Communication Infrastructure (ez_pnp2.py) ---

def test_migration_node_initialization():
    content = read_file(FILE_MAIN)
    # Ensure rclpy is used and rospy is purged
    assert "rclpy.init" in content
    assert "rclpy.create_node" in content
    assert "import rospy" not in content, "Legacy 'rospy' import found in ROS 2 migration"

def test_migration_service_clients():
    content = read_file(FILE_MAIN)
    # Check for ROS 2 client factory instead of ROS 1 ServiceProxy
    assert re.search(r"create_client\s*\(", content)
    assert "AddToDatabase" in content
    assert "GetPositionIK" in content

## --- 2. Toolset & Logic Migration (ez_tools.py) ---

def test_migration_tf2_logic():
    content = read_file(FILE_TOOLS)
    # ROS 2 uses tf2_ros.Buffer and TransformListener
    assert "tf2_ros.Buffer()" in content
    assert "lookup_transform" in content
    # Check for ROS 2 specific time/duration objects
    assert "rclpy.time.Time()" in content or "Duration" in content

def test_migration_async_handling():
    content = read_file(FILE_TOOLS)
    # ROS 2 services are typically handled via futures
    assert "call_async" in content
    assert "spin_until_future_complete" in content

def test_migration_kinematic_persistence():
    content = read_file(FILE_TOOLS)
    # Ensure MoveIt collision scene updates are migrated
    assert ".attach_object(" in content, "Missing attach_object call for gripper link"
    assert ".detach_object(" in content, "Missing detach_object call after placement"

def test_migration_scaling_logic():
    content = read_file(FILE_TOOLS)
    # GraspIt works in mm, MoveIt in meters. 1000x factor is mandatory.
    assert "pose_factor = 1000" in content
    assert re.search(r"\*\s*self\.pose_factor", content)

## --- 3. Integration Integrity (test2_ez_pnp2.py) ---

def test_integration_logic_params():
    content = read_file(FILE_TEST)
    # Verify the test targets the correct object 'Z'
    assert re.search(r"graspit_target_object\s*=\s*[\"']Z[\"']", content)
    # Verify the test initializes the ROS 2 node correctly
    assert "rclpy.create_node" in content
