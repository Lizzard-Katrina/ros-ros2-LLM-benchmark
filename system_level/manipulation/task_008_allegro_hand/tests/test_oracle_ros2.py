import re
from pathlib import Path
# Mock function for file reading
def get_content(file_path):
    # In a real benchmark environment, this reads the student's submitted file
    with open(file_path, 'r') as f:
        return f.read()

DRV_CPP = Path(__file__).resolve().parents[1]/"AllegroHandDrv.cpp"
NODE_CPP = Path(__file__).resolve().parents[1]/"allegro_node.cpp"

# --- Test 1: CAN Unpacking Logic (Driver Layer) ---
def test_can_unpack_logic():
    content = get_content(DRV_CPP)
    # 1. Matches data[any_index] | (data[any_index] << 8)
    # 2. Handles optional & 0xFF masks and varied spacing
    patterns = [
        r"data\[.*?\]\s*(\||\+)\s*\(?\s*data\[.*?\]\s*<<\s*8\s*\)?", 
        r"0\.088",                                            
        r"M_PI\s*/\s*180\.0"                                  
    ]
    for p in patterns:
        assert re.search(p, content), f"Missing core unpacking logic or scaling factor: {p}"
# --- Test 2: Bitmask Readiness State Machine (Driver Layer) ---
def test_bitmask_update():
    content = get_content(DRV_CPP)
    # Ensure _curr_position_get is updated using the finger index bit-shift
    # Expected: _curr_position_get |= (0x01 << findex);
    pattern = r"_curr_position_get\s*\|?=\s*\(\s*0x01\s*<<\s*\(?\s*findex\s*\)?\s*\)"
    assert re.search(pattern, content), "The readiness bitmask (_curr_position_get) must be updated correctly."

# --- Test 3: Synchronous Control Flow (Controller Layer) ---
def test_control_sync_flow():
    content = get_content(NODE_CPP)
    # Critical System Sequence: Read -> Fetch -> Compute -> Write -> Reset
    # 1. Check if joint info is retrieved from driver
    # 2. Check if the control law is executed
    # 3. Check if the bitmask is reset (Mandatory to avoid loop freeze)
    flow = [
        r"canDevice->getJointInfo",
        r"computeDesiredTorque\(",
        r"canDevice->resetJointInfoReady\("
    ]
    for f in flow:
        assert re.search(f, content), f"Missing essential step in the control loop sequence: {f}"

# --- Test 4: Cross-Cycle State Backup (Controller Layer) ---
def test_data_consistency_backup():
    content = get_content(NODE_CPP)
    # Check if current_position is backed up to previous_position 
    # before the new CAN data overwrites it.
    pattern = r"previous_position\[.*?\]\s*=\s*current_position\[.*?\]"
    assert re.search(pattern, content), "Must backup current_position before updating new sensor data."

# --- Test 5: Velocity Derivation (System Correctness) ---
def test_velocity_calculation():
    content = get_content(NODE_CPP)
    # Verify that velocity is derived using finite difference with the control interval 'dt'
    pattern = r"\(\s*current_position\[.*?\]\s*-\s*previous_position\[.*?\]\s*\)\s*/\s*dt"
    assert re.search(pattern, content), "Velocity must be calculated as (curr - prev) / dt."

# --- Test 6: Hardware-Level Safety Interlock (System Level) ---
def test_emergency_stop_logic():
    content = get_content(NODE_CPP)
    # Check if the node reacts to the driver's error status (lEmergencyStop < 0)
    # and triggers a clean ROS shutdown.
    pattern = r"if\s*\(\s*lEmergencyStop\s*<\s*0\s*\).*?ros::shutdown\(\)"
    assert re.search(pattern, content, re.DOTALL), "The node must shutdown immediately if the driver reports an emergency status."
# --- Test 7: Hardware Flag Consistency (System Variable Check) ---
def test_variable_naming_consistency():
    content = get_content(DRV_CPP)
    # The driver uses 'HAND_TYPE_A' (bool) or 'RIGHT_HAND' (bool) 
    # instead of custom '_hand_type' char/string.
    # Check if the LLM used the correct member variable from the header.
    patterns = [
        r"if\s*\(\s*HAND_TYPE_A\s*\)",  # Correct boolean flag
        r"if\s*\(\s*RIGHT_HAND\s*\)"
    ]
    # If the LLM used _hand_type (which doesn't exist in the provided header snippets), 
    # this test will fail.
    found_correct_flag = any(re.search(p, content) for p in patterns)
    assert found_correct_flag, "LLM used an undefined variable (e.g., _hand_type). Use 'HAND_TYPE_A' from the driver header."
