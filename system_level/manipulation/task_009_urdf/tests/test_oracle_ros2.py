import re
from pathlib import Path

# File path definitions
URDF_FILE = Path(__file__).resolve().parents[1] / "arm_urdf.urdf"
SRDF_FILE = Path(__file__).resolve().parents[1] / "manipulator.srdf"
LIMITS_FILE = Path(__file__).resolve().parents[1] / "joint_limits.yaml"

def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

def test_urdf_kinematics():
    content = get_content(URDF_FILE)
    
    # 1. Verify Joint 2 rotation axis (Pitch axis)
    # Uses [^>]* to skip potential attributes or newlines within the tag
    j2_axis = r'<joint name="joint2"[^>]*>.*?<axis xyz="0 1 0"'
    assert re.search(j2_axis, content, re.DOTALL), "Joint2 must rotate around the Y-axis (0 1 0)."

    # 2. Verify Link 3 cylinder center offset logic
    # Checks if the model correctly calculated the 0.15 midpoint for a 0.3m cylinder
    l3_origin = r'<link name="link3"[^>]*>.*?<origin[^>]*xyz="0 0 0\.15"'
    assert re.search(l3_origin, content, re.DOTALL), "Link3 visual/collision origin must be (0 0 0.15) to align cylinder base."

    # 3. Verify Joint 3 spatial topology
    # Calculation: 0.3m (link length) - 0.02m (joint offset) = 0.28m
    j3_origin = r'<joint name="joint3"[^>]*>.*?<origin[^>]*xyz="0 0 0\.28"'
    assert re.search(j3_origin, content, re.DOTALL), "Joint3 origin must be (0 0 0.28) to connect at the end of Link2."

def test_srdf_semantic_logic():
    content = get_content(SRDF_FILE)
    
    # 1. Verify Arm group uses a robust Chain definition
    arm_chain = r'<group name="arm">.*?<chain base_link="base_link" tip_link="link6"'
    assert re.search(arm_chain, content, re.DOTALL), "The 'arm' group should be defined as a kinematic chain from base_link to link6."

    # 2. Verify Allowed Collision Matrix (ACM) adjacent link disabling
    # Must include at least the link5 and link6 adjacent pair
    acm_check = r'<disable_collisions link1="link5" link2="link6" reason="Adjacent"'
    assert re.search(acm_check, content), "Adjacent links (link5, link6) must have collision checking disabled."

def test_joint_limits_overrides():
    content = get_content(LIMITS_FILE)
    
    # 1. Verify explicit activation of velocity limit boolean flags
    # MoveIt ignores numerical limits if the boolean 'has_velocity_limits' is not 'true'
    vel_limit_flag = r'joint1:.*?has_velocity_limits:\s*true'
    assert re.search(vel_limit_flag, content, re.DOTALL), "Velocity limits for joint1 must be explicitly enabled (true)."

    # 2. Verify fine-grained acceleration limits for gripper joints
    f_joint_acc = r'f_joint1:.*?has_acceleration_limits:\s*true.*?max_acceleration:\s*0\.5'
    assert re.search(f_joint_acc, content, re.DOTALL), "Finger joints must have acceleration limits enabled and set to 0.5."
