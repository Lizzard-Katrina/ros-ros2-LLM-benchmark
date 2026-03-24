import re
from pathlib import Path

# If your harness uses a different filename (e.g., code.cpp), change this.
CPP_FILE = Path(__file__).resolve().parents[1] / "process_image.cpp"
FLAGS = re.MULTILINE | re.DOTALL


def _read() -> str:
    assert CPP_FILE.exists(), f"Expected C++ file at: {CPP_FILE}"
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")


def _has(pat: str, s: str) -> bool:
    return re.search(pat, s, FLAGS) is not None


def _assert_has(pat: str, s: str, msg: str):
    if not _has(pat, s):
        raise AssertionError(msg + f"\nMissing pattern:\n{pat}")


def _assert_not_has(pat: str, s: str, msg: str):
    if _has(pat, s):
        raise AssertionError(msg + f"\nForbidden pattern found:\n{pat}")


# -------------------------
# ORACLE (merged, less redundant)
# NOTE: test_09 is preserved (do not modify).
# -------------------------

def test_01_ros2_not_ros1_and_has_core_headers():
    """Concept: Uses ROS2 rclcpp + ROS2 Image; does not use ROS1 ros::."""
    s = _read()
    _assert_has(r'#include\s*[<"]\s*rclcpp/rclcpp\.hpp\s*[>"]', s,
                "Expected ROS2 include rclcpp/rclcpp.hpp (either <...> or \"...\").")
    _assert_has(r'#include\s*[<"]\s*sensor_msgs/msg/image\.hpp\s*[>"]', s,
                "Expected ROS2 include sensor_msgs/msg/image.hpp.")
    _assert_not_has(r'#include\s*[<"]\s*ros/ros\.h\s*[>"]', s,
                    "Found ROS1 header ros/ros.h; expected ROS2-only.")
    _assert_not_has(r"\bros::(init|NodeHandle|spin|Subscriber|ServiceClient)\b", s,
                    "Found ROS1 ros:: API usage; expected ROS2 rclcpp APIs.")


def test_02_node_spin_and_image_subscription_pipeline():
    """
    Merged concept (old test_03 + test_04):
    - Node created and spun (rclcpp::init + rclcpp::spin)
    - Subscribes to sensor_msgs::msg::Image on an image topic
    - Attaches a callback (bind or lambda)
    """
    s = _read()
    _assert_has(r"\brclcpp::init\s*\(", s, "Expected rclcpp::init(...) in main.")
    _assert_has(r"\brclcpp::spin\s*\(", s, "Expected rclcpp::spin(...) to run the node.")
    _assert_has(r"\brclcpp::shutdown\s*\(", s, "Expected rclcpp::shutdown(...) for clean shutdown.")

    _assert_has(r":\s*public\s+rclcpp::Node\b|std::make_shared\s*<\s*\w+\s*>", s,
                "Expected a ROS2 Node (subclass of rclcpp::Node or constructed via std::make_shared<...>()).")

    _assert_has(r"create_subscription\s*<\s*sensor_msgs::msg::Image\s*>", s,
                "Expected create_subscription<sensor_msgs::msg::Image>(...).")
    _assert_has(r'create_subscription[^;]*\(\s*"[^"]*image[^"]*"\s*,', s,
                "Expected subscription topic string containing 'image' (camera image topic).")
    _assert_has(r"create_subscription[^;]*,\s*(std::bind|\[[^\]]*\]\s*\()", s,
                "Expected a callback attached to subscription (std::bind(...) or lambda [...](...){...}).")


def test_03_drive_service_client_request_fields_and_send():
    """
    Merged concept (old test_05 + test_06):
    - Creates DriveToTarget service client
    - Sets BOTH linear and angular request fields
    - Sends request via async_send_request
    """
    s = _read()
    _assert_has(r"create_client\s*<\s*[^>]*DriveToTarget[^>]*\s*>", s,
                "Expected create_client<...DriveToTarget...>(...) (service client).")
    _assert_has(r"(request\s*(->|\.)\s*(linear_x|linear)\b)\s*=\s*[^;]+;", s,
                "Expected assignment to request linear component (e.g., request->linear_x = ...).")
    _assert_has(r"(request\s*(->|\.)\s*(angular_z|angular)\b)\s*=\s*[^;]+;", s,
                "Expected assignment to request angular component (e.g., request->angular_z = ...).")
    _assert_has(r"async_send_request\s*\(", s,
                "Expected async_send_request(...) (service request actually sent).")


def test_04_perception_scans_image_data_buffer():
    """Concept: Scans through the image buffer (loop over data) rather than a fixed pixel."""
    s = _read()
    _assert_has(r"(\.|->)\s*data\s*\[", s,
                "Expected access to image data buffer (msg.data[...] or msg->data[...]).")
    _assert_has(r"\bfor\s*\([^)]*\)\s*\{[^}]*data\s*\[", s,
                "Expected a for-loop that iterates while reading data[...] (blob scan).")


def test_05_rgb_triplet_check_present():
    """
    Concept: RGB-triplet style checks (i, i+1, i+2) combined in a condition.
    Does NOT require literal 255.
    """
    s = _read()
    _assert_has(r"data\s*\[\s*\w+\s*\+\s*1\s*\]", s,
                "Expected adjacent channel access like data[i+1] (RGB triplet).")
    _assert_has(r"data\s*\[\s*\w+\s*\+\s*2\s*\]", s,
                "Expected adjacent channel access like data[i+2] (RGB triplet).")
    _assert_has(r"\bif\s*\([^)]*&&[^)]*&&[^)]*\)", s,
                "Expected combined channel condition (e.g., cond1 && cond2 && cond3) to detect blob.")


def test_06_three_region_left_center_right_decision_with_geometry_thresholds():
    """
    Concept: left/center/right 3-way decision using image geometry.
    Requires:
    - if / else if / else
    - uses width or step
    - contains two threshold-ish cutpoints (~1/3 and ~2/3), but not exact formula.
    """
    s = _read()
    _assert_has(r"\bif\s*\([^)]*\)\s*\{", s, "Expected an if-branch for region decision.")
    _assert_has(r"\belse\s+if\s*\([^)]*\)\s*\{", s, "Expected an else-if branch (3-way split).")
    _assert_has(r"\belse\s*\{", s, "Expected a final else branch (3-way split).")
    _assert_has(r"\b(width|step)\b", s,
                "Expected using image width/step for left/center/right reasoning.")

    cut_hits = len(re.findall(r"/\s*3\b|2\s*\*\s*\w+\s*/\s*3\b|\b0\.33\b|\b0\.66\b", s, FLAGS))
    if cut_hits < 2:
        raise AssertionError(
            "Expected two cutpoints for 3-region split (roughly 1/3 and 2/3 of width/step). "
            "Did not find enough threshold-like expressions."
        )


# -------------------------


def test_07_stop_and_motion_semantics_without_literal_numbers():
    """
    Tight semantic similarity to ROS1 reference, but avoids overfitting to literal numeric constants.

    Requires evidence of:
    - detection indicator (found/sentinel/etc.)
    - motion mapping: assigns linear component somewhere AND assigns angular component with BOTH signs (+ and -) somewhere
    - stop behavior: explicit zeros OR implicit zeros via zero-initialized vars copied into request
    """
    s = _read()

    # Detection indicator (broad)
    _assert_has(
        r"\b(found|detected|seen|pixel|blob|target)\b|\b-1\s*;",
        s,
        "Expected a detection indicator (found flag / pixel/blob variable / sentinel -1)."
    )

    # Motion evidence (not tied to literal numbers):
    # A) linear component is assigned (request linear or twist linear)
    _assert_has(
        r"(request\s*(->|\.)\s*(linear_x|linear)\b|linear\.\w+)\s*=\s*[^;]+;",
        s,
        "Expected linear motion mapping: assign to request linear_x/linear (or Twist linear.*)."
    )

    # B) angular component is assigned with both signs somewhere.
    # We accept either:
    #   angular_z = -expr  AND angular_z = expr
    # or: expr negated in one assignment and not negated in another.
    _assert_has(
        r"(request\s*(->|\.)\s*(angular_z|angular)\b|angular\.\w+)\s*=\s*-\s*[^;]+;",
        s,
        "Expected turning mapping: an assignment that sets angular component to a NEGATED expression (turn one direction)."
    )
    _assert_has(
        r"(request\s*(->|\.)\s*(angular_z|angular)\b|angular\.\w+)\s*=\s*(?!-)[^;]+;",
        s,
        "Expected turning mapping: an assignment that sets angular component without leading '-' (opposite direction or neutral)."
    )

    # Stop evidence:
    # Option A: explicit zeros on both components (request or twist)
    has_explicit_stop = (
        _has(r"(linear_x|linear\.\w+)\s*=\s*0(\.0+)?f?\s*;", s) and
        _has(r"(angular_z|angular\.\w+)\s*=\s*0(\.0+)?f?\s*;", s)
    )

    # Option B: implicit stop via zero-initialized variables copied into request fields
    # Look for two vars initialized to 0, and later both request fields assigned from identifiers.
    has_zero_init_pair = _has(
        r"\b(double|float|auto)\s+\w+\s*=\s*0(\.0+)?f?\s*;\s*\b(double|float|auto)\s+\w+\s*=\s*0(\.0+)?f?\s*;",
        s
    )
    has_req_from_vars = _has(
        r"(request\s*(->|\.)\s*(linear_x|linear)\b)\s*=\s*\w+\s*;\s*.*(request\s*(->|\.)\s*(angular_z|angular)\b)\s*=\s*\w+\s*;",
        s
    )
    has_implicit_stop = has_zero_init_pair and has_req_from_vars

    # Option C: stop via a helper call (0,0)
    has_stop_call = _has(r"\b\w+\s*\(\s*0(\.0+)?f?\s*,\s*0(\.0+)?f?\s*\)\s*;", s)

    if not (has_explicit_stop or has_implicit_stop or has_stop_call):
        raise AssertionError(
            "Expected stop-when-not-found behavior evidence.\n"
            "Acceptable patterns:\n"
            "- explicit: set both linear and angular to 0\n"
            "- implicit: velocities initialized to 0 and copied into request fields\n"
            "- helper call: some_function(0, 0)\n"
        )
