import re
import pytest
from pathlib import Path

# File paths for the migrated ROS 2 scripts
CTRL_FILE = Path(__file__).resolve().parents[1]/"controller.py"
MP_FILE = Path(__file__).resolve().parents[1]/"motion_planning.py"

def get_content(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

## --- Controller.py Verification ---

def test_controller_move_to_logic():
    """
    Verifies that move_to implements complex interpolation instead of 
    simple point-to-point jumps.
    """
    content = get_content(CTRL_FILE)
    
    # Check for core mathematical interpolation: Slerp and numpy stepping
    assert "slerp" in content.lower(), "Logic Failure: Missing orientation interpolation (Slerp)."
    assert "np.arange" in content or "range" in content, "Logic Failure: Missing step-based trajectory generation."
    
    # Verify internal state update (gripper_pose)
    assert re.search(r"self\.gripper_pose\s*=", content), \
        "State Failure: Controller must update internal gripper_pose after motion."

def test_ros2_trajectory_interface():
    """
    Verifies the use of ROS 2 Action Clients or correct Topic interfaces.
    """
    content = get_content(CTRL_FILE)
    
    # Verify message types are updated to ROS 2 style
    assert "trajectory_msgs.msg" in content, "Import Failure: Missing trajectory_msgs."
    
    # Verify usage of ActionClient (Best Practice) or at least create_publisher
    # ROS 2 should move away from rospy.Publisher style
    communication_valid = any(x in content for x in ["ActionClient", "create_client", "create_publisher"])
    assert communication_valid, "Interface Failure: No valid ROS 2 communication method found in Controller."

## --- Motion_planning.py Verification ---

def test_manipulation_orchestration_logic():
    """
    Verifies the integrity of the manipulation loop in Hole 2.
    """
    content = get_content(MP_FILE)
    
    # 1. Verify sequence: Get Pose -> Straighten -> Move -> Place -> Fix
    critical_steps = ["straighten", "move_to", "open_gripper", "set_model_fixed"]
    for step in critical_steps:
        assert step in content, f"Orchestration Failure: Missing critical step '{step}' in manipulation loop."

    # 2. Verify Gazebo link logic (Attach/Detach)
    # ROS 2 uses service clients with .call() or .call_async()
    assert "attach_srv" in content and "detach_srv" in content, \
        "Linkage Failure: Model did not implement block attachment/detachment logic."

def test_coordinate_transformation_consistency():
    """
    Verifies coordinate calculation logic migration.
    """
    content = get_content(MP_FILE)
    
    # Check for interlocking height compensation
    assert "INTERLOCKING_OFFSET" in content, "Precision Failure: Missing interlocking height compensation."
    
    # Check for target pose orientation math
    assert "DEFAULT_QUAT" in content and "PyQuaternion" in content, \
        "Kinematics Failure: Missing orientation math during target pose calculation."

def test_system_error_handling():
    """
    Verifies handling of invalid model names in the loop.
    """
    content = get_content(MP_FILE)
    
    # Ensure the loop skips unrecognized models gracefully
    assert "continue" in content and ("ValueError" in content or "except" in content), \
        "Robustness Failure: System must 'continue' the loop if a model is not recognized."

## --- Cross-file Consistency ---

def test_cross_file_instantiation():
    """
    Verifies interface matching between the controller and planner.
    """
    ctrl_content = get_content(CTRL_FILE)
    mp_content = get_content(MP_FILE)
    
    assert "class ArmController" in ctrl_content
    assert "ArmController(" in mp_content, "Linkage Failure: ArmController instantiation mismatch in planner."

def test_no_nested_deadlocks():
    """
    CRITICAL: Detects if the model uses spin_until_future_complete inside class methods.
    In ROS 2, spinning inside a callback/method of a node that is already being 
    managed by an executor will lead to a recursive spin error or a permanent deadlock.
    """
    files_to_check = [CTRL_FILE, MP_FILE]
    
    for file_path in files_to_check:
        content = get_content(file_path)
        lines = content.splitlines()
        
        in_method = False
        for line_num, line in enumerate(lines, 1):
            clean_line = line.strip()
            if clean_line.startswith("def ") and "self" in clean_line:
                in_method = True
                continue
            if in_method and "rclpy.spin_until_future_complete" in clean_line:
                if not clean_line.startswith("#"):
                    assert False, (
                        f"Architecture Failure in {file_path.name} at line {line_num}: "
                        f"Detected 'spin_until_future_complete' inside a method. "
                        "This will cause deadlocks. Use async/await or callbacks."
                    )
            if in_method and len(line) > 0 and not line.startswith(" "):
                in_method = False
def test_trajectory_interpolation_integrity():
    """
    Verifies that the model didn't 'cheat' by removing the motion loop.
    A simple move_to that only sends one point is physically dangerous.
    """
    content = get_content(CTRL_FILE)
    
    # 1. Check for the existence of a loop that iterates through steps
    has_loop = re.search(r"for\s+.*?\s+in\s+(np\.arange|range)", content)
    # 2. Check if it actually interpolates the orientation
    has_slerp = "slerp" in content.lower()
    
    assert has_loop and has_slerp, (
        "Kinematics Failure: The move_to implementation is missing smooth interpolation. "
        "You must iterate through steps and use Slerp to ensure safe arm movement."
    )

def test_strict_naming_compliance():
    """
    Ensures the model followed the specific naming constraints for service clients.
    """
    content = get_content(MP_FILE)
    required_names = ["self.setstatic_srv", "self.attach_srv", "self.detach_srv"]
    
    for name in required_names:
        assert name in content, f"Naming Failure: Service client must be named exactly '{name}'."
