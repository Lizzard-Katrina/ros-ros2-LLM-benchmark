import re
import pytest
from pathlib import Path

# Paths to the three coupled files
BASE_PATH = Path(__file__).resolve().parents[1]
MANAGER_FILE = BASE_PATH / "manage_objects_node.py"
BEHAVIOR_FILE = BASE_PATH / "pickup_behaviors_node.py"
CONTROLLER_FILE = BASE_PATH / "turtlebot_controller_node.py"

@pytest.fixture
def manager_code():
    return MANAGER_FILE.read_text() if MANAGER_FILE.exists() else ""

@pytest.fixture
def behavior_code():
    return BEHAVIOR_FILE.read_text() if BEHAVIOR_FILE.exists() else ""

@pytest.fixture
def controller_code():
    return CONTROLLER_FILE.read_text() if CONTROLLER_FILE.exists() else ""

## --- 1. System Coordination: Service Interface Matching ---

def test_service_interface_alignment(manager_code, behavior_code):
    """Verify that the Manager and Behavior Tree use the same ROS 2 service names."""
    # Check if Manager defines the service
    # Expected: create_service(Trigger, 'check_object', ...)
    srv_definition = re.search(r'create_service\(\s*Trigger\s*,\s*[\'"](~?check_object)[\'"]', manager_code)
    # Check if Behavior creates the client
    # Expected: create_client(Trigger, '/manage_objects/check_object')
    cli_definition = re.search(r'create_client\(\s*Trigger\s*,\s*[\'"](/manage_objects/check_object|check_object)[\'"]', behavior_code)
    
    assert srv_definition and cli_definition, "System Desync: Service name mismatch between Manager (Server) and Behavior (Client)."

## --- 2. Gazebo Migration: SpawnEntity Implementation ---

def test_gazebo_spawn_migration(manager_code):
    """Verify that SpawnModel (ROS 1) is migrated to SpawnEntity (ROS 2)."""
    assert "SpawnEntity" in manager_code, "API Error: Found legacy SpawnModel. Must use SpawnEntity in ROS 2 Gazebo."
    assert not re.search(r'SpawnModel', manager_code), "Leakage Error: Legacy SpawnModel service still present."
    # Check for async call which is standard in ROS 2
    assert re.search(r'\.call_async\(', manager_code), "Architecture Error: Gazebo spawn call should be asynchronous."

## --- 3. Behavior Tree: Async-to-Sync Bridge ---

def test_bt_future_handling(behavior_code):
    """Verify the BT node handles ROS 2 futures to avoid blocking the tick."""
    # Searching for future handling patterns (spin_until_future_complete or checking future.done())
    sync_pattern = r'(?:spin_until_future_complete|future\.result|future\.done)'
    assert re.search(sync_pattern, behavior_code), \
        "Logic Error: Behavior Tree update() must handle ROS 2 futures to remain non-blocking yet functional."

## --- 4. Global Namespace: Leading Slash Removal ---

def test_no_leading_slashes_in_topics(manager_code, behavior_code, controller_code):
    """Verify system-wide adherence to ROS 2 topic/frame naming (no leading slashes)."""
    # Specifically checking common culprits like /odom, /cmd_vel, /gazebo
    slash_pattern = r'[\'"]/(?:odom|cmd_vel|gazebo|spawn_entity|manage_objects)'
    
    assert not re.search(slash_pattern, manager_code), "Static Consistency: Leading slash found in Manager topics."
    assert not re.search(slash_pattern, behavior_code), "Static Consistency: Leading slash found in Behavior topics."
    assert not re.search(slash_pattern, controller_code), "Static Consistency: Leading slash found in Controller topics."

## --- 5. Lifecycle: Timer Migration ---

def test_timer_migration_pattern(controller_code):
    """Verify migration from rospy.Timer to Node.create_timer."""
    # Expected: self.create_timer(0.1, self.controller)
    assert re.search(r'create_timer\(\s*(?:0\.1|1\.0/10\.0)\s*,\s*(?:self\.)?controller\)', controller_code), \
        "Lifecycle Error: Controller must use Node.create_timer() instead of rospy.Timer."

## --- 6. Semantic Consistency: Blackboard Interaction ---

def test_bt_blackboard_usage(behavior_code):
    """Verify the Behavior Tree correctly writes to the Blackboard using ROS 2 patterns."""
    # Check for writing the result message to the blackboard
    # Expected: self.blackboard.object_name = resp.message or similar
    bb_pattern = r'blackboard\.(?:object_name|set\(["\']object_name["\'])\s*='
    assert re.search(bb_pattern, behavior_code), "Logic Error: BT Node failed to store 'object_name' in the Blackboard."

## --- 7. Anti-Leakage: Global Search for rospy ---

def test_absolute_no_rospy(manager_code, behavior_code, controller_code):
    """Ensure the entire system is free of ROS 1 rospy artifacts."""
    combined_code = manager_code + behavior_code + controller_code
    assert "import rospy" not in combined_code, "System Leakage: 'import rospy' found in migrated code."
    assert "rospy." not in combined_code, "System Leakage: 'rospy' calls found in migrated code."

## 8.
def test_no_nested_spin(manager_code):
    assert not re.search(r'def\s+\w+\(self.*:\s*.*?rclpy\.spin', manager_code, re.DOTALL)
## 9.
def test_qos_usage(controller_code):
    assert re.search(r'create_(?:publisher|subscription)\(.*,\s*(?:10|QoSProfile)', controller_code)
