import re
from pathlib import Path

# -------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------

TRANSLATED_FILE = "offboard_control.cpp"

def load_code():
    """
    Load the translated ROS2 C++ code produced by the LLM.
    This is the target of the oracle, NOT the original ROS1 code.
    """
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()

# -------------------------------------------------------------------
# 1. ROS1 while + Rate  → ROS2 timer (control loop semantics)
# -------------------------------------------------------------------

def test_ros1_control_loop_migrated_to_ros2_timer():
    """
    Semantic equivalence:
    ROS1 `while (ros::ok()) { ... rate.sleep(); }`
    must be migrated to a ROS2 timer-based execution model.
    """
    code = load_code()

    # ROS1-style control loop must not exist
    assert "while (ros::ok()" not in code
    assert "ros::Rate" not in code

    # ROS2 timer must exist
    assert re.search(r"create_.*timer", code), \
        "ROS1 control loop not migrated to ROS2 timer"


# -------------------------------------------------------------------
# 2. Control logic must live INSIDE the timer callback
# -------------------------------------------------------------------

def test_control_logic_inside_timer_callback():
    """
    Semantic equivalence:
    Periodic PX4 control logic must execute inside the timer callback,
    not only once in constructor or main().
    """
    code = load_code()

    timer_cb = re.search(
        r"create_.*timer\s*\([^)]*\)\s*,\s*\[this\]\s*\(\)\s*\{([\s\S]*?)\}",
        code
    )

    assert timer_cb, "Timer callback not found"

    body = timer_cb.group(1)

    # Must publish PX4-related messages periodically
    assert "publish" in body, \
        "No publish call inside timer callback (control logic missing)"


    # Ensure constructor does not directly call publish
    ctor = re.search(r"OffboardControl\s*\(\)\s*:\s*Node\([^)]*\)\s*\{([\s\S]*?)\}", code)
    if ctor:
        ctor_body = ctor.group(1)
        assert "publish(" not in ctor_body, \
            "Control logic executed in constructor instead of timer"

# -------------------------------------------------------------------
# 3. PX4 offboard heartbeat semantics (continuous streaming)
# -------------------------------------------------------------------

def test_offboard_heartbeat_semantics_preserved():
    """
    Semantic equivalence:
    PX4 offboard mode requires continuous streaming of setpoints.
    This must be implemented as a periodic ROS2 timer.
    """
    code = load_code()

    # Must publish offboard-related messages
    assert re.search(r"OffboardControlMode|TrajectorySetpoint", code), \
        "No PX4 offboard setpoint messages found"

    # Must be periodic
    assert re.search(r"create_.*timer", code), \
        "Offboard heartbeat is not periodic (no timer found)"


# -------------------------------------------------------------------
# 4. PX4 command ordering semantics (setpoint → offboard → arm)
# -------------------------------------------------------------------

def test_px4_command_order_semantics():
    """
    Semantic equivalence:
    ROS1 PX4 offboard logic requires the following order:
      1. stream setpoints
      2. switch to OFFBOARD mode
      3. arm the vehicle
    """
    code = load_code()

    setpoint_idx = code.find("TrajectorySetpoint")
    offboard_idx = code.find("VEHICLE_CMD_DO_SET_MODE")
    arm_idx = code.find("VEHICLE_CMD_COMPONENT_ARM_DISARM")

    assert setpoint_idx != -1, "No setpoint command found"
    assert offboard_idx != -1, "No OFFBOARD mode command found"
    assert arm_idx != -1, "No ARM command found"

    assert setpoint_idx < offboard_idx < arm_idx, \
        "PX4 command order violates ROS1 offboard semantics"


# -------------------------------------------------------------------
# 5. ROS time semantics (ros::Time::now → ROS2 clock)
# -------------------------------------------------------------------

def test_ros_time_semantics_preserved():
    """
    Semantic equivalence:
    ROS1 `ros::Time::now()` must map to ROS2 clock-based time.
    Wall-clock or std::chrono-based timestamps are NOT acceptable.
    """
    code = load_code()

    forbidden = [
        "std::chrono",
        "system_clock",
        "steady_clock"
    ]

    for f in forbidden:
        assert f not in code, \
            f"Wall-clock time '{f}' breaks ROS time semantics"

    assert re.search(r"get_clock|now\(", code), \
        "No ROS2 clock-based timestamp found"


# -------------------------------------------------------------------
# 6. PX4 QoS semantics must be explicit
# -------------------------------------------------------------------

def test_px4_qos_semantics_explicit():
    """
    Semantic equivalence:
    PX4 uORB <-> ROS2 bridge relies on explicit QoS semantics.
    Default QoS is insufficient.
    """
    code = load_code()

    assert "QoS" in code, \
        "QoS not explicitly configured"

    assert re.search(r"best_effort|reliable", code), \
        "QoS reliability not specified"




# -------------------------------------------------------------------
# 7. No ROS1 API leakage
# -------------------------------------------------------------------

def test_no_ros1_api_leakage():
    """
    Semantic hygiene:
    Translated ROS2 code must not retain ROS1 APIs.
    """
    code = load_code()

    forbidden = [
        "ros::NodeHandle",
        "ros::Publisher",
        "ros::spin",
        "ros::Time",
        "ros::ok"
    ]

    for f in forbidden:
        assert f not in code, f"ROS1 API leaked into ROS2 code: {f}"
