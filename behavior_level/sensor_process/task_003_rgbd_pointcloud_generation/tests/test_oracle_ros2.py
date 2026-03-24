import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] /"point_cloud_xyzrgb.cpp"

def get_content():
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    return re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.DOTALL)

def test_intrinsic_scaling_math():
    """Concept: Physical Correctness. Must scale focal lengths (fx/fy) when resizing."""
    content = get_content()
    # Check for ratio calculation and actual multiplication of intrinsics
    # Matches: k[0] *= ratio, fx *= ratio, K[0] *= ratio, etc.
    scaling_pattern = r"(?:\.k|k|p|P|fx|fy|K)\[?[0-9]?\]?\s*\*=\s*ratio"
    assert re.search(scaling_pattern, content), \
        "Failure: Scaling Defect. You resized the image but didn't scale the focal length (fx/fy). The 3D world will be distorted."

def test_offset_variable_usage():
    """Concept: Style Compliance. Identify offsets instead of using cv::cvtColor."""
    content = get_content()
    # Enforces the 'red_offset', 'blue_offset' variables as requested in TODO
    assert "red_offset" in content and "blue_offset" in content, \
        "Failure: Style Violation. You must define red/blue offsets to handle different encodings manually."

def test_mandatory_kernel_call():
    """Concept: Library Integration. Must use convertDepth and convertRgb kernels."""
    content = get_content()
    # Ensures the model uses the provided library functions instead of a custom loop
    assert "convertDepth" in content and "convertRgb" in content, \
        "Failure: Instruction Violation. You must call the 'convertDepth' and 'convertRgb' kernels."

def test_header_and_frame_sync():
    """Concept: Coordinate System Integrity. Frame_id must come from Depth sensor."""
    content = get_content()
    # Checks for full header assignment to ensure frame_id and stamp are identical
    assert re.search(r"cloud_msg->header\s*=\s*depth_msg->header", content), \
        "Failure: Sync Error. The PointCloud must inherit the header from the depth_msg for TF accuracy."

def test_memory_unique_ownership():
    """Concept: Modern C++ Standards. Use unique_ptr and move semantics."""
    content = get_content()
    # Verify unique_ptr and the move operation for publishing
    assert "std::make_unique" in content, "Failure: Use std::make_unique for memory efficiency."
    assert "std::move" in content, "Failure: You must move the unique_ptr to the publisher."

def test_dispatch_logic_16uc1_32fc1():
    """Concept: Type Awareness. Must handle both depth pixel formats."""
    content = get_content()
    # Checks if both encodings are handled via branching
    assert "TYPE_16UC1" in content and "TYPE_32FC1" in content, \
        "Failure: Incomplete support. You must handle both 16-bit and 32-bit depth encodings."
