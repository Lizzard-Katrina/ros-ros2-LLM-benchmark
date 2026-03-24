import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] /"ukf.cpp"

def get_content():
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    # Remove comments for clean matching
    return re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.DOTALL)

def test_3d_rotation_coupling():
    """Concept: 3D Projection. Validates RZYX mapping for X-axis."""
    content = get_content()
    # A correct 3D X-projection must involve both yaw (cy) and pitch (cp)
    assert re.search(r"cy\s*\*\s*cp\s*\*\s*delta_sec", content), \
        "Failure: Under-determined Physics. X-axis projection must account for both Yaw and Pitch."

def test_acceleration_3d_projection():
    """Concept: Force/Acceleration Projection. New check for 3D Accel mapping."""
    content = get_content()
    # Checks if the model projects Body-frame Accel using the same 3D rotation as Velocity
    # Look for 0.5 * (rotation terms) * dt^2
    accel_pattern = r"0\.5\s*\*.*?(?:cy|cp|sy|sp).*?delta_sec\s*\*\s*delta_sec"
    assert re.search(accel_pattern, content), \
        "Failure: Kinematic Error. Acceleration must be rotated into the global frame using the same RZYX logic."

def test_angular_singularity_mapping():
    """Concept: Euler Derivatives. Checks for the 1/cos(pitch) terms."""
    content = get_content()
    # cpi is the standard 'robot_localization' variable for 1/cos(pitch)
    # tp is the standard for tan(pitch)
    assert any(x in content for x in ["cpi", "tp", "1.0 / cp", "1.0/cp"]), \
        "Failure: Mathematical Singularity. Missing 1/cos(pitch) mapping for angular rates."

def test_index_safety_enforcement():
    """Concept: Architecture Compliance. Ban hard-coded indices."""
    content = get_content()
    # Check for hard-coded matrix access like (0, 3) or [0][3]
    hardcoded_pattern = r"transfer_function_\(\s*\d+\s*,\s*\d+\s*\)"
    if re.search(hardcoded_pattern, content):
        assert False, "Failure: Architecture Violation. Do not use hard-coded integers (0, 1, 2) for state indices. Use StateMember enum."

def test_eigen_optimization():
    """Concept: Performance optimization."""
    content = get_content()
    assert "applyOnTheLeft" in content, \
        "Failure: Performance Violation. Use 'sigma_point.applyOnTheLeft(transfer_function_)' to prevent temporary object creation."

def test_state_member_coverage():
    """Concept: Domain Knowledge. Ensure key state members are used."""
    content = get_content()
    members = ["StateMemberVx", "StateMemberVpitch", "StateMemberAx"]
    for m in members:
        assert m in content, f"Failure: Implementation did not reference mandatory state member {m}."
