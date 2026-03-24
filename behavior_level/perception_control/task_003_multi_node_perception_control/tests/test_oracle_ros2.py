

import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "kobuki_joystick.cpp"

def _code() -> str:
    assert CPP_FILE.exists(), f"Expected C++ file at {CPP_FILE}, but it does not exist."
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")


def _assert_has(pattern: str, msg: str):
    code = _code()
    if re.search(pattern, code, flags=re.MULTILINE | re.DOTALL) is None:
        raise AssertionError(msg + f"\nMissing pattern:\n{pattern}")


def _assert_not_has(pattern: str, msg: str):
    code = _code()
    if re.search(pattern, code, flags=re.MULTILINE | re.DOTALL) is not None:
        raise AssertionError(msg + f"\nForbidden pattern found:\n{pattern}")


# 1) ROS2-only + main lifecycle (init/spin/shutdown) + NO ROS1 surface
def test_ros2_only_and_lifecycle():
    _assert_has(r'#include\s*[<"]rclcpp/rclcpp\.hpp[>"]',
                "Must include rclcpp/rclcpp.hpp (ROS2).")
    _assert_has(r'RCLCPP_(?:INFO|WARN|ERROR|DEBUG)(?:_STREAM)?\s*\(',
                "Must use ROS2 logging macros (RCLCPP_*).")

    _assert_has(r'rclcpp::init\s*\(',
                "Must call rclcpp::init(...) in main().")
    _assert_has(
        r'(rclcpp::spin(?:_some|_once)?\s*\()|'
        r'(rclcpp::executors::\w+)|'
        r'(\b\w+\s*(?:\.|\->)\s*spin(?:_some|_once)?\s*\()',
        "Must spin node/executor (rclcpp::spin/spin_some or executor.spin)."
    )
    _assert_has(r'rclcpp::shutdown\s*\(',
                "Must call rclcpp::shutdown().")

    _assert_not_has(
        r'#include\s*[<"]ros/ros\.h[>"]|'
        r'\bros::NodeHandle\b|'
        r'\bros::init\s*\(|'
        r'\bros::ok\s*\(|'
        r'\bros::Rate\b|'
        r'ROS_(?:INFO|WARN|ERROR|DEBUG)(?:_STREAM)?\s*\(',
        "Must not contain ROS1 roscpp surface (ros/ros.h, ros::NodeHandle, ROS_* logs, etc.)."
    )


# 2) Linux joystick driver fidelity: open nonblocking + js_event + read() + JS_EVENT_INIT cleared
def test_linux_js_event_io_fidelity():
    _assert_has(r'#include\s*<fcntl\.h>', "Must include <fcntl.h> (reference uses open flags).")
    _assert_has(r'#include\s*<unistd\.h>', "Must include <unistd.h> (reference uses read()).")

    _assert_has(
        r'\bopen\s*\(\s*[^;]*\|\s*O_NONBLOCK',
        "Must open joystick with O_NONBLOCK (open(... | O_NONBLOCK)).",
    )
    _assert_has(r'\bjs_event\b', "Must use Linux joystick struct js_event (reference behavior).")
    _assert_has(
        r'\bread\s*\(\s*[^,]+,\s*&\s*\w+,\s*sizeof\s*\(\s*\w+\s*\)\s*\)',
        "Must read joystick events with read(fd, &event, sizeof(event)) (reference behavior).",
    )

    # You explicitly want this strict: must clear init flag like reference
    _assert_has(
        r'event\s*\.\s*type\s*&=\s*~\s*JS_EVENT_INIT',
        "Must clear JS_EVENT_INIT from event.type exactly like reference: event.type &= ~JS_EVENT_INIT;",
    )


# 3) DS4 enums fidelity: DS4_BUTTONS + DS4_AXIS with the same key symbols used in code
def test_ds4_enum_mapping_present_and_used():
    # Require the enum blocks exist and contain the reference labels
    _assert_has(
        r'enum\s+DS4_BUTTONS\s*{[^}]*\bL1\b[^}]*}',
        "Must define enum DS4_BUTTONS containing L1 (reference mapping).",
    )
    _assert_has(
        r'enum\s+DS4_AXIS\s*{[^}]*\bL3_Y\b[^}]*\bR3_X\b[^}]*}',
        "Must define enum DS4_AXIS containing L3_Y and R3_X (reference mapping).",
    )

    # Must use JS_EVENT_BUTTON/AXIS constants and compare event.number against DS4 enums
    _assert_has(
        r'event\s*\.\s*type\s*==\s*JS_EVENT_BUTTON',
        "Must handle JS_EVENT_BUTTON events (reference behavior).",
    )
    _assert_has(
        r'event\s*\.\s*type\s*==\s*JS_EVENT_AXIS',
        "Must handle JS_EVENT_AXIS events (reference behavior).",
    )
    _assert_has(
        r'event\s*\.\s*number\s*==\s*DS4_BUTTONS::L1',
        "Must map L1 button via event.number == DS4_BUTTONS::L1 (reference behavior).",
    )
    _assert_has(
        r'event\s*\.\s*number\s*==\s*DS4_AXIS::L3_Y',
        "Must map linear control via event.number == DS4_AXIS::L3_Y (reference behavior).",
    )
    _assert_has(
        r'event\s*\.\s*number\s*==\s*DS4_AXIS::R3_X',
        "Must map angular control via event.number == DS4_AXIS::R3_X (reference behavior).",
    )


# 4) L1 enable/disable fidelity: value==1 enable sets enabled true, value==0 disable sets enabled false
def test_l1_toggle_semantics_fidelity():
    # Require press/release checks
    _assert_has(
        r'event\s*\.\s*value\s*==\s*1',
        "Must check button-press event.value == 1 (reference enable on press).",
    )
    _assert_has(
        r'event\s*\.\s*value\s*==\s*0',
        "Must check button-release event.value == 0 (reference disable on release).",
    )
    # Require enabled state flips (we accept either m_enabled or enabled)
    _assert_has(
        r'(m_enabled|enabled)\s*=\s*true',
        "Must set enabled state true on L1 press (reference behavior).",
    )
    _assert_has(
        r'(m_enabled|enabled)\s*=\s*false',
        "Must set enabled state false on L1 release (reference behavior).",
    )

    # Require MotorPower ON/OFF semantics (strongly tied to reference)
    _assert_has(
        r'MotorPower\s*::\s*ON|\bstate\b\s*=\s*.*\bON\b',
        "Must publish MotorPower ON when enabling (reference behavior).",
    )
    _assert_has(
        r'MotorPower\s*::\s*OFF|\bstate\b\s*=\s*.*\bOFF\b',
        "Must publish MotorPower OFF when disabling (reference behavior).",
    )

    # Require stop Twist on disable
    _assert_has(r'linear\s*\.\s*x\s*=\s*0(\.0+)?\s*;', "Disable must set linear.x = 0 (stop).")
    _assert_has(r'angular\s*\.\s*z\s*=\s*0(\.0+)?\s*;', "Disable must set angular.z = 0 (stop).")


# 5) Axis scaling + gated publish fidelity: "-event.value/32767 * scale_*" and publish only when enabled AND nonzero
def test_axis_scaling_and_publish_gating_fidelity():
    # Require the 32767 normalization constant
    _assert_has(r'32767(?:\.0+)?', "Must normalize using 32767 (reference int16 range).")

    # Require assignments shaped like reference: linear.x = -event.value / 32767.0 * scale_linear
    # Allow minor whitespace/casts, but require: linear.x assign uses event.value, a leading '-', '/ 32767', '* scale_linear'
    _assert_has(
        r'linear\s*\.\s*x\s*=\s*-\s*[^;]*event\s*\.\s*value[^;]*/\s*32767(?:\.0+)?[^;]*\*\s*[^;]*scale_linear',
        "linear.x must be computed from -event.value/32767 * scale_linear (reference scaling).",
    )
    _assert_has(
        r'angular\s*\.\s*z\s*=\s*-\s*[^;]*event\s*\.\s*value[^;]*/\s*32767(?:\.0+)?[^;]*\*\s*[^;]*scale_angular',
        "angular.z must be computed from -event.value/32767 * scale_angular (reference scaling).",
    )

    # Require gated publish like reference: if (enabled && (linear.x != 0 || angular.z != 0)) publish(twist)
    _assert_has(
        r'if\s*\(\s*[^)]*\b(m_enabled|enabled)\b[^)]*&&\s*\(\s*[^)]*linear\s*\.\s*x\s*!=\s*0[^)]*\|\|[^)]*angular\s*\.\s*z\s*!=\s*0[^)]*\)\s*\)',
        "cmd_vel must be published only when enabled AND (linear.x != 0 OR angular.z != 0) (reference gating).",
    )
    _assert_has(
        r'create_publisher\s*<\s*geometry_msgs::msg::Twist\s*>\s*\(\s*["\']cmd_vel["\']',
        "Must publish geometry_msgs::msg::Twist to 'cmd_vel' (ROS2 topic fidelity).",
    )
