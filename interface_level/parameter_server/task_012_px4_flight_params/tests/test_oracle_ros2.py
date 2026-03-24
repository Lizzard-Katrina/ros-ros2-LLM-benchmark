import re
from pathlib import Path

# Path to the python script
PY_FILE = Path(__file__).resolve().parents[1] / "offboard_control.py"

def get_content():
    with open(PY_FILE, 'r') as f:
        content = f.read()
    # Pre-process: remove comments to prevent false positives
    content = re.sub(r'#.*', '', content)
    return content

def test_parameter_declaration_logic():
    """Concept: Use ROS 2 rclpy API to declare flight-critical parameters."""
    content = get_content()
    # Check for declaration of takeoff height and yaw
    # Matches: self.declare_parameter('name', default_value)
    assert re.search(r"self\.declare_parameter\s*\(\s*['\"]takeoff_height['\"]", content), \
        "Failure: 'takeoff_height' parameter was not declared using self.declare_parameter()."
    assert re.search(r"self\.declare_parameter\s*\(\s*['\"]target_yaw['\"]", content), \
        "Failure: 'target_yaw' parameter was not declared using self.declare_parameter()."

def test_parameter_value_retrieval():
    """Concept: Correctly retrieve values from the Parameter Server into class attributes."""
    content = get_content()
    # Matches: self.get_parameter(...).value OR self.get_parameter(...).get_parameter_value().double_value
    # We look for the '.value' access which is standard in rclpy
    assert re.search(r"self\..*=\s*self\.get_parameter\(.*?\)\.(?:value|get_parameter_value)", content), \
        "Failure: Parameters were declared but not retrieved and assigned to class attributes (self.xxx)."

def test_dynamic_message_population_yaw():
    """Concept: TrajectorySetpoint message must use the dynamic yaw parameter."""
    content = get_content()
    # Ensure the yaw field in the message is not hardcoded
    # Correct: msg.yaw = self.target_yaw  (or similar attribute)
    # Incorrect: msg.yaw = 1.57079
    assert re.search(r"msg\.yaw\s*=\s*self\.", content), \
        "Failure: TrajectorySetpoint.yaw is still using a hardcoded value or not assigned from a class attribute."
    assert not re.search(r"msg\.yaw\s*=\s*1\.57079", content), \
        "Failure: Hardcoded yaw value (1.57079) detected in message population."

def test_px4_msg_field_integrity():
    """Concept: TrajectorySetpoint uses a 'position' array, not x, y, z fields."""
    content = get_content()
    assert not re.search(r"msg\.[xyz]\s*=", content), \
        "Failure: TrajectorySetpoint uses 'position' array. Do not use 'msg.x', 'msg.y', or 'msg.z'."
    assert "msg.position" in content, \
        "Failure: Must use 'msg.position' to set 3D coordinates in PX4 messages."


def test_dynamic_message_population_position():
    """Concept: TrajectorySetpoint position must use the dynamic height parameter."""
    content = get_content()
    # Ensure the Z-axis of position list uses the attribute
    # Matches: msg.position = [x, y, self.takeoff_height]
    assert re.search(r"msg\.position\s*=\s*\[.*,.*,.*self\..*\]", content), \
        "Failure: TrajectorySetpoint.position[2] (Z-axis) must use the dynamic height attribute."

def test_px4_timestamping_standard():
    """Concept: PX4 messages require microsecond timestamps."""
    content = get_content()
    # PX4 uses microseconds. ROS 2 nanoseconds must be divided by 1000.
    assert re.search(r"nanoseconds\s*/\s*1000", content) or "int(" in content, \
        "Failure: PX4 message timestamping must be converted to microseconds (nanoseconds / 1000)."

def test_logging_for_safety():
    """Concept: Critical flight parameters should be logged for operator verification."""
    content = get_content()
    assert re.search(r"self\.get_logger\(\)\.info\(", content), \
        "Failure: Flight parameters should be logged using get_logger().info() during initialization."

def test_no_hardcoded_flight_constants():
    """Concept: Ensure legacy hardcoded flight values are completely removed."""
    content = get_content()
    # Check that -5.0 (height) and 1.57079 (yaw) are no longer being assigned directly to msg or variables
    assert not re.search(r"=\s*-5\.0", content), "Failure: Hardcoded height (-5.0) still present in assignment logic."
    # We allow them in declare_parameter as defaults, so we specifically check assignment/message logic
    # Note: Regex can be refined if defaults are allowed, but usually we want them gone from the loop.
