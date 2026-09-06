import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "turtlesim.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()

## --- Tests ---

def test_uses_ros2_declare_parameter():
    content = get_content()
    pattern = r"nh_->declare_parameter\s*(?:<.*?>)?\s*\("
    assert re.search(pattern, content), \
        "Critical Error: Missing 'declare_parameter'. ROS2 requires declaration before use."

def test_enforces_vector_int64_structure():
    content = get_content()
    pattern = r"std::vector\s*<\s*int64_t\s*>"
    assert re.search(pattern, content), \
        "Constraint Violation: Must use 'std::vector<int64_t>' for atomic RGB configuration."

def test_correct_parameter_naming():
    content = get_content()
    assert '"background_color_rgb"' in content or "'background_color_rgb'" in content, \
        "Semantic Error: Parameter must be named 'background_color_rgb'."

def test_default_value_integrity():
    content = get_content()
    pattern = r"\{\s*69\s*,\s*86\s*,\s*255\s*\}"
    assert re.search(pattern, content), \
        "Logic Error: Default fallback values {69, 86, 255} (Deep Sky Blue) not found."

def test_no_ros1_nodehandle_residue():
    content = get_content()
    bad_patterns = [r"\.getParam\(", r"ros::NodeHandle"]
    for pattern in bad_patterns:
        assert not re.search(pattern, content), \
            f"Legacy Error: Found ROS1 syntax '{pattern}'. Migration is incomplete."

def test_type_strictness_casting():
    content = get_content()
    pattern = r"(?:auto|vector).*?=\s*nh_->declare_parameter"
    assert re.search(pattern, content), \
        "Interface Error: Should assign the result of declare_parameter to a variable for immediate use."