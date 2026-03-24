import re
import pytest


from pathlib import Path

# -------------------------------------------------------------------
# Utility
# -------------------------------------------------------------------

TRANSLATED_FILE ="amcl_node.cpp"

def load_code():
    """
    Load the translated ROS2 C++ code produced by the LLM.
    This is the target of the oracle, NOT the original ROS1 code.
    """
    file_path = Path(__file__).parent.parent / TRANSLATED_FILE
    assert file_path.exists(), f"Translated ROS2 file not found: {TRANSLATED_FILE}"
    return file_path.read_text()


@pytest.fixture
def code():
    return load_code()
# =============================
# Concept-based Oracle Tests
# =============================

def test_action_server_creation(code):
    """Check if a ROS2 Action Server is defined."""
    pattern = r"rclcpp_action::create_server<\s*UpdatePose\s*>"
    assert re.search(pattern, code), \
        "Missing ROS2 Action Server creation for UpdatePose action."

def test_handle_goal_defined(code):
    """Check if handle_goal callback exists."""
    pattern = r"handle_goal\s*\("
    assert re.search(pattern, code), \
        "handle_goal callback function is not implemented."

def test_handle_cancel_defined(code):
    """Check if handle_cancel callback exists."""
    pattern = r"handle_cancel\s*\("
    assert re.search(pattern, code), \
        "handle_cancel callback function is not implemented."

def test_handle_accepted_defined(code):
    """Check if handle_accepted callback exists."""
    pattern = r"handle_accepted\s*\("
    assert re.search(pattern, code), \
        "handle_accepted callback function is not implemented."

def test_todo_comment_present(code):
    """Check if TODO for particle filter update is present."""
    pattern = r"//\s*TODO: Implement particle filter update"
    assert re.search(pattern, code), \
        "Missing TODO comment for particle filter update."

def test_laser_data_handling_mentioned(code):
    """Check if code mentions laser data ranges/bearings."""
    pattern = r"(ranges|bearing)"
    assert re.search(pattern, code), \
        "Laser sensor ranges or bearings handling not mentioned."

def test_resample_called(code):
    """Check if particle filter resampling is indicated."""
    pattern = r"(resample|pf_update_resample)"
    assert re.search(pattern, code), \
        "Particle filter resampling step not found."

def test_pose_publishing_mentioned(code):
    """Check if publishing pose is mentioned."""
    pattern = r"(publish|pose_pub_|\bPoseWithCovarianceStamped\b)"
    assert re.search(pattern, code), \
        "Publishing of pose not implemented or mentioned."

def test_thread_usage_for_async(code):
    """Check if async handling (std::thread) is used in handle_accepted."""
    pattern = r"std::thread\s*\("
    assert re.search(pattern, code), \
        "Async thread handling for action execution missing."

def test_feedback_or_result_mentioned(code):
    """Check if action result or feedback is defined."""
    pattern = r"(Feedback|Result|goal_handle->succeed)"
    assert re.search(pattern, code), \
        "Action feedback/result handling missing in handle_accepted."
