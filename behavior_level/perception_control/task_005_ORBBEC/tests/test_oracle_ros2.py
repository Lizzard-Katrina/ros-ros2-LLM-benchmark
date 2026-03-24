import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "ob_camera_node.cpp"




def read_code() -> str:
    assert CPP_FILE.exists(), f"CPP file not found at {CPP_FILE}"
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")


def extract_function_body(code: str, func_name: str) -> str:
    """
    Extract function body for 'OBCameraNode::func_name(...) { ... }'
    Tolerates ROS2 signatures and whitespace.
    """
    sig = re.search(
        rf"(?:^|\n)\s*.*?\bOBCameraNode::{re.escape(func_name)}\s*\([^)]*\)\s*\{{",
        code,
        flags=re.MULTILINE,
    )
    assert sig, f"Missing function definition: OBCameraNode::{func_name}(...)"
    start = sig.end()

    # Find next member function definition as an approximate end boundary
    tail = code[start:]
    nxt = re.search(
        r"\n\s*.*?\bOBCameraNode::\w+\s*\([^)]*\)\s*\{",
        tail,
        flags=re.MULTILINE,
    )
    end = start + (nxt.start() if nxt else len(tail))
    return code[start:end]


def assert_has(body: str, pattern: str, msg: str):
    if re.search(pattern, body, flags=re.DOTALL | re.MULTILINE) is None:
        raise AssertionError(msg + f"\nExpected pattern:\n{pattern}")


def assert_not_has(body: str, pattern: str, msg: str):
    if re.search(pattern, body, flags=re.DOTALL | re.MULTILINE) is not None:
        raise AssertionError(msg + f"\nUnexpected pattern:\n{pattern}")


def assert_any(body: str, patterns: list[str], msg: str):
    for p in patterns:
        if re.search(p, body, flags=re.DOTALL | re.MULTILINE):
            return
    raise AssertionError(msg + "\nExpected one of patterns:\n" + "\n".join(patterns))

#0) syntax

def test_no_ros1_core_runtime_api_left():
    """
    Concept: translated code should not rely on ROS1 runtime APIs/types/macros.
    This is a syntax/compatibility guard, not behavior.
    """
    code = read_code()

    # Hard ROS1 leftovers that should not appear in ROS2 code.cpp
    banned = [
        r"\bros::NodeHandle\b",
        r"\bros::Time\b",
        r"\bros::Duration\b",
        r"\bros::Timer\b",
        r"\bros::ok\s*\(",
        r"\bros::spin\s*\(",
        r"\bros::Rate\b",
        r"\bROS_INFO\b|\bROS_WARN\b|\bROS_ERROR\b|\bROS_DEBUG\b",  # ROS1 logging macros
        r"\bnodelet\b",  # common ROS1 pattern
        r"\bmessage_filters::Subscriber\b",  # typical ROS1 include usage
    ]
    for pat in banned:
        assert_not_has(
            code,
            pat,
            "ROS2 translated code should not contain ROS1 runtime APIs/macros. Found ROS1 pattern.",
        )


def test_ros2_core_presence_minimum():
    """
    Concept: require some ROS2 signatures so we don't accept 'ROS1 code renamed' accidentally.
    """
    code = read_code()

    # At least one of these should exist in a ROS2 node implementation
    assert_any(
        code,
        [
            r"\brclcpp::Node\b",
            r"\brclcpp::ok\s*\(",
            r"\bRCLCPP_(?:INFO|WARN|ERROR|DEBUG)",
            r"\bsensor_msgs::msg::",
            r"\bgeometry_msgs::msg::",
            r"\btf2_ros::",
        ],
        "ROS2 translated code should show ROS2 core usage (rclcpp/RCLCPP_* msgs/tf2_ros).",
    )

# ---------------------------------------
# 1) onNewFrameSetCallback: big loop semantics
# ---------------------------------------

def test_frameset_retrieves_multimodal_frames_semantically():
    """
    Concept: The frameset callback must retrieve/obtain depth, color, left/right color, left/right IR
    in SOME way (API may differ across ROS2 wrapper versions).
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    # Depth
    assert_any(
        body,
        [
            r"\bdepth_frame\b",
            r"depthFrame\s*\(",
            r"getFrame\s*\(\s*OB_FRAME_DEPTH\s*\)",
            r"getFrame\s*\(\s*.*DEPTH.*\)",
        ],
        "onNewFrameSetCallback should obtain a depth frame (depth_frame / depthFrame() / getFrame(...DEPTH...)).",
    )

    # Color
    assert_any(
        body,
        [
            r"\bcolor_frame\b",
            r"colorFrame\s*\(",
            r"getFrame\s*\(\s*OB_FRAME_COLOR\s*\)",
            r"getFrame\s*\(\s*.*COLOR.*\)",
        ],
        "onNewFrameSetCallback should obtain a color frame (color_frame / colorFrame() / getFrame(...COLOR...)).",
    )

    # Left/Right color (semantic)
    assert_any(
        body,
        [
            r"\bleft_?color_?frame\b",
            r"getFrame\s*\(\s*OB_FRAME_COLOR_LEFT\s*\)",
            r"getFrame\s*\(\s*.*COLOR_LEFT.*\)",
            r"\bCOLOR_LEFT\b",
        ],
        "onNewFrameSetCallback should obtain left color stream frame (left_color_frame / ...COLOR_LEFT...).",
    )
    assert_any(
        body,
        [
            r"\bright_?color_?frame\b",
            r"getFrame\s*\(\s*OB_FRAME_COLOR_RIGHT\s*\)",
            r"getFrame\s*\(\s*.*COLOR_RIGHT.*\)",
            r"\bCOLOR_RIGHT\b",
        ],
        "onNewFrameSetCallback should obtain right color stream frame (right_color_frame / ...COLOR_RIGHT...).",
    )

    # Left/Right IR (semantic)
    assert_any(
        body,
        [
            r"\bleft_?ir_?frame\b",
            r"getFrame\s*\(\s*OB_FRAME_IR_LEFT\s*\)",
            r"getFrame\s*\(\s*.*IR_LEFT.*\)",
            r"\bIR_LEFT\b",
        ],
        "onNewFrameSetCallback should obtain left IR frame (...IR_LEFT...).",
    )
    assert_any(
        body,
        [
            r"\bright_?ir_?frame\b",
            r"getFrame\s*\(\s*OB_FRAME_IR_RIGHT\s*\)",
            r"getFrame\s*\(\s*.*IR_RIGHT.*\)",
            r"\bIR_RIGHT\b",
        ],
        "onNewFrameSetCallback should obtain right IR frame (...IR_RIGHT...).",
    )


def test_frameset_applies_filters_before_downstream_use():
    """
    Concept: depth/color/IR frames should be passed through filter pipelines when present.
    Don't require exact API; require calls to the filter processing helpers.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_has(
        body,
        r"processDepthFrameFilter\s*\(",
        "onNewFrameSetCallback should call processDepthFrameFilter(...) on depth frames.",
    )
    assert_has(
        body,
        r"processColorFrameFilter\s*\(",
        "onNewFrameSetCallback should call processColorFrameFilter(...) on color frames (at least once).",
    )
    assert_any(
        body,
        [
            r"processLeftIrFrameFilter\s*\(",
            r"processRightIrFrameFilter\s*\(",
            r"process(?:Left|Right)IrFrameFilter\s*\(",
        ],
        "onNewFrameSetCallback should call processLeft/RightIrFrameFilter(...) when IR frames exist.",
    )

    # Do not hard-require pushFrame (ROS2 wrappers may differ),
    # but require SOME indication that processed frame is used afterwards:
    assert_any(
        body,
        [
            r"pushFrame\s*\(",
            r"frame_set\s*=\s*\w+",      # replaced frameset
            r"frame_set->\w+\s*\(",      # further usage of frameset after processing
        ],
        "onNewFrameSetCallback should use the processed frames downstream (push back OR update frameset variable OR continue pipeline).",
    )


def test_frameset_alignment_semantics_present():
    """
    Concept: when depth registration/alignment is enabled, an align filter should be applied to the frameset.
    Accept different condition forms/variable names, but require align_filter_ use.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    # Must mention align_filter_ and process call
    assert_has(
        body,
        r"\balign_filter_\b",
        "onNewFrameSetCallback should reference align_filter_ (alignment filter) when alignment is supported.",
    )
    assert_has(
        body,
        r"align_filter_->process\s*\(",
        "onNewFrameSetCallback should call align_filter_->process(...) to perform alignment when enabled.",
    )

    # Condition should include depth_registration_ OR alignment enable flag
    assert_any(
        body,
        [
            r"\bdepth_registration_\b",
            r"\balign_mode_\b",
            r"\benable_d2c_viewer_\b",
        ],
        "onNewFrameSetCallback should guard alignment with a configuration flag (e.g., depth_registration_/align_mode_/enable_d2c_viewer_).",
    )


def test_frameset_dispatches_to_color_queue_or_falls_back_to_pointcloud_publish():
    """
    Concept: if color stream enabled, frameset is queued for a consumer thread; otherwise publish point cloud directly.
    Allow notify_one/notify_all.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    assert_has(
        body,
        r"colorFrameQueue_\.push\s*\(",
        "onNewFrameSetCallback should enqueue framesets for color processing (colorFrameQueue_.push(...)).",
    )
    assert_any(
        body,
        [
            r"colorFrameCV_\.notify_all\s*\(",
            r"colorFrameCV_\.notify_one\s*\(",
        ],
        "onNewFrameSetCallback should notify a waiting color thread after enqueue (notify_one or notify_all).",
    )

    # Fallback publish when not going through color queue
    assert_has(
        body,
        r"publishPointCloud\s*\(",
        "onNewFrameSetCallback should publish point clouds somewhere in the frameset callback path (publishPointCloud(...)).",
    )


def test_frameset_forwards_non_color_streams_through_common_handler():
    """
    Concept: iterate over streams and forward non-color frames to onNewFrameCallback,
    including IR MJPG decode attempt.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameSetCallback")

    # Loop over IMAGE_STREAMS (allow different variable name)
    assert_any(
        body,
        [
            r"for\s*\([^)]*:\s*IMAGE_STREAMS\s*\)",
            r"for\s*\([^)]*IMAGE_STREAMS[^)]*\)",
        ],
        "onNewFrameSetCallback should iterate over IMAGE_STREAMS (range-for) to forward frames.",
    )

    # Skip color types concept
    assert_any(
        body,
        [
            r"OB_FRAME_COLOR",
            r"COLOR_LEFT",
            r"COLOR_RIGHT",
        ],
        "onNewFrameSetCallback should treat color frames specially (skip/continue them in the non-color forwarding loop).",
    )

    # IR MJPG decode attempt + forwarding to common handler
    assert_has(
        body,
        r"decodeIRMJPGFrame\s*\(",
        "onNewFrameSetCallback should attempt to decode IR MJPG frames (decodeIRMJPGFrame(...)) before forwarding.",
    )
    assert_has(
        body,
        r"onNewFrameCallback\s*\(",
        "onNewFrameSetCallback should forward frames to the shared handler (onNewFrameCallback(...)).",
    )


# ---------------------------------------
# 2) onNewColorFrameCallback: consumer thread semantics
# ---------------------------------------

def test_color_thread_uses_condition_variable_wait_with_predicate():
    """
    Concept: thread should use condition_variable wait (or wait_for) with predicate
    that wakes on queue non-empty OR shutdown.
    """
    code = read_code()
    body = extract_function_body(code, "onNewColorFrameCallback")

    assert_any(
        body,
        [
            r"colorFrameCV_\.wait\s*\(\s*lock\s*,",
            r"colorFrameCV_\.wait_for\s*\(\s*lock\s*,",
        ],
        "onNewColorFrameCallback should wait on condition variable (wait or wait_for), not busy-loop.",
    )

    # Predicate concept: queue non-empty OR !is_running_
    assert_any(
        body,
        [
            r"!\s*colorFrameQueue_\.empty\s*\(\s*\).*?\|\|.*?!\s*\(?\s*is_running_",
            r"!\s*is_running_.*?\|\|.*?!\s*colorFrameQueue_\.empty",
        ],
        "onNewColorFrameCallback wait predicate should wake when queue is non-empty OR shutdown requested.",
    )

    # Negative: avoid pure sleep polling loops
    assert_not_has(
        body,
        r"sleep_for\s*\(",
        "onNewColorFrameCallback should not rely on sleep_for polling; it should block on condition_variable.",
    )


def test_color_thread_consumes_fifo_and_processes_in_pipeline_order():
    """
    Concept: FIFO consume frameset -> decode -> publishPointCloud -> forward to onNewFrameCallback
    """
    code = read_code()
    body = extract_function_body(code, "onNewColorFrameCallback")

    # FIFO: front + pop (or move+pop)
    assert_any(
        body,
        [
            r"colorFrameQueue_\.front\s*\(",
            r"auto\s+\w+\s*=\s*colorFrameQueue_\.front\s*\(",
        ],
        "onNewColorFrameCallback should take next item via front() from colorFrameQueue_.",
    )
    assert_has(
        body,
        r"colorFrameQueue_\.pop\s*\(",
        "onNewColorFrameCallback should pop() the queue after taking the front item.",
    )

    # Decode concept
    assert_has(
        body,
        r"decodeColorFrameToBuffer\s*\(",
        "onNewColorFrameCallback should decode the color frame into an RGB buffer (decodeColorFrameToBuffer(...)).",
    )

    # Order: decode -> publishPointCloud -> onNewFrameCallback (allow some interleaving but preserve sequence)
    assert_has(
        body,
        r"decodeColorFrameToBuffer[\s\S]*publishPointCloud\s*\([\s\S]*\)[\s\S]*onNewFrameCallback\s*\(",
        "onNewColorFrameCallback should decode, then publish point cloud, then forward frame to onNewFrameCallback.",
    )


# ---------------------------------------
# 3) onNewFrameCallback: per-frame publish semantics
# ---------------------------------------

def test_single_frame_has_subscriber_gate_includes_image_and_camerainfo():
    """
    Concept: work should be gated by subscriber presence (image or camera info).
    Metadata gating may be implemented differently; don't require exact container name.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    assert_has(
        body,
        r"getNumSubscribers\s*\(",
        "onNewFrameCallback should gate processing based on subscriber counts (getNumSubscribers()).",
    )
    assert_any(
        body,
        [
            r"image_publishers_\s*\[",
            r"image_publisher",
            r"image_pub",
        ],
        "onNewFrameCallback should reference an image publisher when gating/publishing images.",
    )
    assert_any(
        body,
        [
            r"camera_info_publishers_\s*\[",
            r"camera_info",
            r"CameraInfo",
        ],
        "onNewFrameCallback should consider publishing camera info when subscribers exist (camera_info...).",
    )


def test_single_frame_uses_timestamp_and_frame_id_semantics():
    """
    Concept: obtain frame timestamp (us) and convert to ROS2 time, set frame_id with depth registration handling.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    assert_has(
        body,
        r"getFrameTimestampUs\s*\(\s*frame\s*\)",
        "onNewFrameCallback should call getFrameTimestampUs(frame) to obtain timestamps consistently with time domain.",
    )
    assert_any(
        body,
        [
            r"fromUsToROSTime\s*\(",
            r"\bTime\b.*from",
            r"rclcpp::Time\s*\(",
        ],
        "onNewFrameCallback should convert timestamps into ROS2 time objects (e.g., fromUsToROSTime or equivalent).",
    )
    assert_any(
        body,
        [
            r"depth_aligned_frame_id_",
            r"optical_frame_id_",
            r"frame_id_",
        ],
        "onNewFrameCallback should set a frame_id for published messages (optical_frame_id_/frame_id_/depth_aligned_frame_id_).",
    )


def test_single_frame_supports_flip_branch_and_depth_scaling_hook():
    """
    Concept: image flip option and depth scaling semantics should exist.
    Accept different publish call styles; require cv::flip and a DEPTH-specific scale/use.
    """
    code = read_code()
    body = extract_function_body(code, "onNewFrameCallback")

    # Flip branch: allow image_flip_ or equivalent config gate + cv::flip
    assert_any(
        body,
        [
            r"\bimage_flip_\b",
            r"\bflip\b.*\bconfig\b",
        ],
        "onNewFrameCallback should use a flip configuration flag (e.g., image_flip_).",
    )
    assert_has(
        body,
        r"cv::flip\s*\(",
        "onNewFrameCallback should perform flipping via cv::flip(...) when flip is enabled.",
    )

    # Depth scaling hook: look for DEPTH branch and getValueScale or multiplication by scale
    assert_any(
        body,
        [
            r"stream_index\s*==\s*DEPTH",
            r"\bDEPTH\b",
        ],
        "onNewFrameCallback should have a DEPTH-specific handling branch (stream_index == DEPTH).",
    )
    assert_any(
        body,
        [
            r"getValueScale\s*\(",
            r"depth_scale",
            r"\*\s*depth_scale",
        ],
        "onNewFrameCallback should incorporate depth scale semantics (getValueScale/depth_scale) when handling DEPTH frames.",
    )
