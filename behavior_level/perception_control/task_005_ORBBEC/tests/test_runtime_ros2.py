"""
Runtime test for task_005_ORBBEC: validates the translated ob_camera_node.cpp
by performing regex-based semantic checks on the actual installed file,
and verifying that the file can be located and read from the built package.

Since this node depends on OrbbecSDK hardware libraries that cannot exist
in a Docker CI environment, we validate the translated source file's
semantic correctness by:
1. Locating the installed .cpp file from the built package
2. Running the same pattern-based checks the oracle uses
3. Verifying ROS2 API usage and absence of ROS1 APIs
"""

import re
import subprocess
import sys
import os
import pytest
from pathlib import Path


def find_cpp_file():
    """Find the ob_camera_node.cpp file from the package."""
    # Try the share directory (installed location)
    result = subprocess.run(
        ["ros2", "pkg", "prefix", "task_005_ORBBEC"],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode == 0:
        prefix = result.stdout.strip()
        installed = Path(prefix) / "share" / "task_005_ORBBEC" / "ob_camera_node.cpp"
        if installed.exists():
            return installed

    # Try source directory
    src_candidates = [
        Path(__file__).resolve().parent / "ob_camera_node.cpp",
        Path(__file__).resolve().parent / "src" / "ob_camera_node.cpp",
    ]
    for c in src_candidates:
        if c.exists():
            return c

    # Search workspace
    ws = Path("/ros2_ws")
    if ws.exists():
        for f in ws.rglob("ob_camera_node.cpp"):
            if "task_005_ORBBEC" in str(f):
                return f

    pytest.fail("Could not find ob_camera_node.cpp from task_005_ORBBEC package")


def read_code():
    cpp_file = find_cpp_file()
    return cpp_file.read_text(encoding="utf-8", errors="ignore")


def extract_function_body(code, func_name):
    sig = re.search(
        rf"(?:^|\n)\s*.*?\bOBCameraNode::{re.escape(func_name)}\s*\([^)]*\)\s*\{{",
        code,
        flags=re.MULTILINE,
    )
    assert sig, f"Missing function definition: OBCameraNode::{func_name}(...)"
    start = sig.end()
    tail = code[start:]
    nxt = re.search(
        r"\n\s*.*?\bOBCameraNode::\w+\s*\([^)]*\)\s*\{",
        tail,
        flags=re.MULTILINE,
    )
    end = start + (nxt.start() if nxt else len(tail))
    return code[start:end]


def assert_has(body, pattern, msg):
    if re.search(pattern, body, flags=re.DOTALL | re.MULTILINE) is None:
        raise AssertionError(msg + f"\nExpected pattern:\n{pattern}")


def assert_not_has(body, pattern, msg):
    if re.search(pattern, body, flags=re.DOTALL | re.MULTILINE) is not None:
        raise AssertionError(msg + f"\nUnexpected pattern:\n{pattern}")


def assert_any(body, patterns, msg):
    for p in patterns:
        if re.search(p, body, flags=re.DOTALL | re.MULTILINE):
            return
    raise AssertionError(msg + "\nExpected one of patterns:\n" + "\n".join(patterns))


# ---- Test: Package exists and can be found ----

def test_package_exists():
    """Verify the package was built and installed."""
    result = subprocess.run(
        ["ros2", "pkg", "prefix", "task_005_ORBBEC"],
        capture_output=True, text=True, timeout=10
    )
    assert result.returncode == 0, "Package task_005_ORBBEC not found in ROS2 workspace"


def test_cpp_file_found():
    """Verify the translated cpp file can be located."""
    cpp_file = find_cpp_file()
    assert cpp_file.exists()
    content = cpp_file.read_text()
    assert len(content) > 1000, "File seems too small to be the full translation"


# ---- Test: No ROS1 APIs ----

def test_no_ros1_apis():
    code = read_code()
    banned = [
        r"\bros::NodeHandle\b",
        r"\bros::Time\b",
        r"\bros::Duration\b",
        r"\bros::Timer\b",
        r"\bros::ok\s*\(",
        r"\bros::spin\s*\(",
        r"\bros::Rate\b",
        r"\bROS_INFO\b|\bROS_WARN\b|\bROS_ERROR\b|\bROS_DEBUG\b",
        r"\bnodelet\b",
    ]
    for pat in banned:
        assert_not_has(code, pat,
            "ROS2 code should not contain ROS1 APIs")


# ---- Test: ROS2 APIs present ----

def test_ros2_apis_present():
    code = read_code()
    assert_any(code, [
        r"\brclcpp::Node\b",
        r"\brclcpp::ok\s*\(",
        r"\bRCLCPP_(?:INFO|WARN|ERROR|DEBUG)",
        r"\bsensor_msgs::msg::",
        r"\bgeometry_msgs::msg::",
        r"\btf2_ros::",
    ], "ROS2 code should show ROS2 core usage")


# ---- Test: FrameSet callback semantics ----

def test_frameset_retrieves_frames():
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    # Depth
    assert_any(body, [
        r"\bdepth_frame\b",
        r"depthFrame\s*\(",
        r"getFrame\s*\(\s*OB_FRAME_DEPTH\s*\)",
    ], "Should obtain depth frame")

    # Color
    assert_any(body, [
        r"\bcolor_frame\b",
        r"colorFrame\s*\(",
        r"getFrame\s*\(\s*OB_FRAME_COLOR\s*\)",
    ], "Should obtain color frame")

    # Left/Right color
    assert_any(body, [
        r"\bleft_?color_?frame\b",
        r"getFrame\s*\(\s*OB_FRAME_COLOR_LEFT\s*\)",
        r"\bCOLOR_LEFT\b",
    ], "Should obtain left color frame")

    assert_any(body, [
        r"\bright_?color_?frame\b",
        r"getFrame\s*\(\s*OB_FRAME_COLOR_RIGHT\s*\)",
        r"\bCOLOR_RIGHT\b",
    ], "Should obtain right color frame")

    # IR
    assert_any(body, [
        r"\bleft_?ir_?frame\b",
        r"getFrame\s*\(\s*OB_FRAME_IR_LEFT\s*\)",
        r"\bIR_LEFT\b",
    ], "Should obtain left IR frame")

    assert_any(body, [
        r"\bright_?ir_?frame\b",
        r"getFrame\s*\(\s*OB_FRAME_IR_RIGHT\s*\)",
        r"\bIR_RIGHT\b",
    ], "Should obtain right IR frame")


def test_frameset_applies_filters():
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_has(body, r"processDepthFrameFilter\s*\(", "Should call processDepthFrameFilter")
    assert_has(body, r"processColorFrameFilter\s*\(", "Should call processColorFrameFilter")
    assert_any(body, [
        r"processLeftIrFrameFilter\s*\(",
        r"processRightIrFrameFilter\s*\(",
    ], "Should call IR frame filters")

    assert_any(body, [
        r"pushFrame\s*\(",
        r"frame_set\s*=\s*\w+",
        r"frame_set->\w+\s*\(",
    ], "Should use processed frames downstream")


def test_frameset_alignment():
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_has(body, r"\balign_filter_\b", "Should reference align_filter_")
    assert_has(body, r"align_filter_->process\s*\(", "Should call align_filter_->process")
    assert_any(body, [
        r"\bdepth_registration_\b",
        r"\balign_mode_\b",
        r"\benable_d2c_viewer_\b",
    ], "Should guard alignment with config flag")


def test_frameset_dispatches_to_color_queue():
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_has(body, r"colorFrameQueue_\.push\s*\(", "Should enqueue for color processing")
    assert_any(body, [
        r"colorFrameCV_\.notify_all\s*\(",
        r"colorFrameCV_\.notify_one\s*\(",
    ], "Should notify color thread")
    assert_has(body, r"publishPointCloud\s*\(", "Should publish point clouds")


def test_frameset_forwards_non_color():
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_any(body, [
        r"for\s*\([^)]*:\s*IMAGE_STREAMS\s*\)",
        r"for\s*\([^)]*IMAGE_STREAMS[^)]*\)",
    ], "Should iterate over IMAGE_STREAMS")

    assert_any(body, [
        r"OB_FRAME_COLOR",
        r"COLOR_LEFT",
        r"COLOR_RIGHT",
    ], "Should treat color frames specially")

    assert_has(body, r"decodeIRMJPGFrame\s*\(", "Should decode IR MJPG frames")
    assert_has(body, r"onNewFrameCallback\s*\(", "Should forward to common handler")


# ---- Test: Color thread consumer ----

def test_color_thread_cv_wait():
    code = read_code()
    body = extract_function_body(code, "onNewColorFrameCallback")

    assert_any(body, [
        r"colorFrameCV_\.wait\s*\(\s*lock\s*,",
        r"colorFrameCV_\.wait_for\s*\(\s*lock\s*,",
    ], "Should wait on condition variable")

    assert_any(body, [
        r"!\s*colorFrameQueue_\.empty\s*\(\s*\).*?\|\|.*?!\s*\(?\s*is_running_",
        r"!\s*is_running_.*?\|\|.*?!\s*colorFrameQueue_\.empty",
    ], "Wait predicate should check queue and shutdown")


def test_color_thread_fifo_pipeline():
    code = read_code()
    body = extract_function_body(code, "onNewColorFrameCallback")

    assert_any(body, [
        r"colorFrameQueue_\.front\s*\(",
    ], "Should take front() from queue")

    assert_has(body, r"colorFrameQueue_\.pop\s*\(", "Should pop() queue")
    assert_has(body, r"decodeColorFrameToBuffer\s*\(", "Should decode color frame")
    assert_has(body,
        r"decodeColorFrameToBuffer[\s\S]*publishPointCloud\s*\([\s\S]*\)[\s\S]*onNewFrameCallback\s*\(",
        "Should decode, then publish point cloud, then forward")


# ---- Test: Single frame publish ----

def test_single_frame_subscriber_gate():
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    assert_has(body, r"getNumSubscribers\s*\(", "Should gate on subscriber count")
    assert_any(body, [
        r"image_publishers_\s*\[",
        r"image_publisher",
    ], "Should reference image publisher")
    assert_any(body, [
        r"camera_info_publishers_\s*\[",
        r"camera_info",
        r"CameraInfo",
    ], "Should consider camera info")


def test_single_frame_timestamp_and_frame_id():
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    assert_has(body, r"getFrameTimestampUs\s*\(\s*frame\s*\)", "Should get frame timestamp")
    assert_any(body, [
        r"fromUsToROSTime\s*\(",
        r"rclcpp::Time\s*\(",
    ], "Should convert to ROS2 time")
    assert_any(body, [
        r"depth_aligned_frame_id_",
        r"optical_frame_id_",
        r"frame_id_",
    ], "Should set frame_id")


def test_single_frame_flip_and_depth_scale():
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    assert_any(body, [
        r"\bimage_flip_\b",
    ], "Should use flip config flag")
    assert_has(body, r"cv::flip\s*\(", "Should call cv::flip")

    assert_any(body, [
        r"stream_index\s*==\s*DEPTH",
        r"\bDEPTH\b",
    ], "Should have DEPTH-specific branch")
    assert_any(body, [
        r"getValueScale\s*\(",
        r"depth_scale",
    ], "Should use depth scale")