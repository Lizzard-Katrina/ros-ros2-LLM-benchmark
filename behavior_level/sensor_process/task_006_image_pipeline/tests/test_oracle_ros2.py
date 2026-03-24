import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "point_cloud_xyzrgb_radial.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()

def test_camera_info_scaling_logic():
    """Verify focal length and principal point scaling. Must match the exact style."""
    content = get_content()
    # Check for specific index scaling as required by TODO
    assert "k[0] *= ratio" in content or "k[0] * ratio" in content
    assert "k[2] *= ratio" in content or "k[2] * ratio" in content
    # Critical: Ensure k[8] is NOT scaled (it should remain 1.0)
    assert "k[8] *= ratio" not in content, "Failure: k[8] (constant 1.0) should not be scaled."

def test_cv_bridge_exception_safety():
    """Ensure image conversion is wrapped in try-catch for ROS 2 stability."""
    content = get_content()
    pattern = r"try\s*\{[\s\S]*cv_bridge::toCv(?:Copy|Share)[\s\S]*\}\s*catch\s*\(\s*cv_bridge::Exception"
    assert re.search(pattern, content), "Failure: Missing try-catch block for cv_bridge conversion."

def test_pointcloud2_field_setup():
    """Verify the use of PointCloud2Modifier for binary field layout."""
    content = get_content()
    assert "sensor_msgs::PointCloud2Modifier" in content
    assert 'setPointCloud2FieldsByString(2, "xyz", "rgb")' in content

def test_template_dispatch_completeness():
    """Check if both 16-bit and 32-bit depth encodings are handled."""
    content = get_content()
    assert "convertDepthRadial<uint16_t>" in content
    assert "convertDepthRadial<float>" in content

def test_ros2_unique_ptr_publish():
    """Validate use of move semantics for publishing unique_ptr messages."""
    content = get_content()
    assert re.search(r"publish\s*\(\s*std::move\s*\(", content), "Failure: Should use std::move to publish the unique_ptr."

def test_rgb_offset_style_compliance():
    """Verify the model used the required variable names for color offsets."""
    content = get_content()
    # These names are now forced by the TODO to avoid OOD (Out of Distribution) matching
    assert "red_offset =" in content
    assert "green_offset =" in content
    assert "blue_offset =" in content

def test_temporal_accuracy():
    """Landed Leak 1: Ensure the pointcloud uses the message timestamp, not just 'now()'."""
    content = get_content()
    # The cloud header stamp must come from the depth_msg
    pattern = r"cloud_msg->header\.stamp\s*=\s*depth_msg->header\.stamp"
    assert re.search(pattern, content) or "cloud_msg->header = depth_msg->header" in content

def test_radial_transform_update_check():
    """Landed Leak 2: Ensure radial transform is only updated when necessary (state check)."""
    content = get_content()
    # Check if there's a conditional check for D, K, width, or height before updating transform_
    assert "if" in content and "transform_" in content
    assert "width_" in content or "height_" in content, "Failure: Should check for dimension changes before re-init."

def test_no_legacy_ros1_api():
    """Ensure absence of ROS 1 legacy namespaces and patterns."""
    content = get_content()
    legacy_symbols = ["ros::Time", "ros::NodeHandle", "ros::Publisher", ".toSec()", "boost::bind"]
    for symbol in legacy_symbols:
        assert symbol not in content, f"Failure: Legacy ROS 1 symbol '{symbol}' detected."
