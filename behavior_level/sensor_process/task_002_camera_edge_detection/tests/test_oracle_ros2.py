import re
from pathlib import Path

# Path to the Python node under test
PY_FILE = Path(__file__).resolve().parents[1] / "camera_edge.py"

def get_content():
    with open(PY_FILE, 'r') as f:
        content = f.read()
    # Remove comments to focus on logic
    content = re.sub(r'#.*', '', content)
    return content

def test_bridge_ingestion_positional():
    """Concept: Ingestion must use positional 'bgr8' as per Style Guide."""
    content = get_content()
    # Matches: imgmsg_to_cv2(data, 'bgr8')
    pattern = r"imgmsg_to_cv2\s*\(\s*\w+\s*,\s*['\"]bgr8['\"]\s*\)"
    assert re.search(pattern, content), \
        "Failure: Style violation. Use positional argument 'bgr8' in imgmsg_to_cv2."

def test_explicit_grayscale_conversion():
    """Concept: (Perception Best Practice) Explicit grayscale conversion."""
    content = get_content()
    # Checks for BGR2GRAY conversion
    assert "COLOR_BGR2GRAY" in content or "cvtColor" in content, \
        "Failure: Missing preprocessing. Images must be converted to grayscale before Canny detection."

def test_bridge_egress_positional():
    """Concept: Egress must use positional 'mono8' as per Style Guide."""
    content = get_content()
    # Matches: cv2_to_imgmsg(edges, 'mono8')
    pattern = r"cv2_to_imgmsg\s*\(\s*\w+\s*,\s*['\"]mono8['\"]\s*\)"
    assert re.search(pattern, content), \
        "Failure: Style violation. Use positional argument 'mono8' in cv2_to_imgmsg."

def test_header_full_assignment():
    """Concept: Sync integrity via full header assignment."""
    content = get_content()
    # Matches: .header = data.header
    pattern = r"\.header\s*=\s*\w+\.header"
    assert re.search(pattern, content), \
        "Failure: Style violation. Use full header assignment (out_msg.header = data.header) for sync."

def test_exception_handling_specific():
    """Concept: Error handling with specific CvBridgeError."""
    content = get_content()
    assert "except CvBridgeError" in content, \
        "Failure: Must explicitly catch 'CvBridgeError' for conversion safety."

def test_no_legacy_rospy():
    """Concept: Ensure no ROS 1 artifacts are used."""
    content = get_content()
    assert "rospy" not in content, "Failure: 'rospy' detected in ROS 2 task."
