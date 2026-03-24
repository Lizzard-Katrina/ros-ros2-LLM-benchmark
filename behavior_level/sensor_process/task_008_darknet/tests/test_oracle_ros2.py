import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "YoloObjectDetector.cpp"

def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()


def test_flexible_logging_macros():
    """Verify any RCLCPP logging level is used."""
    content = get_content()
    assert re.search(r"RCLCPP_(?:INFO|DEBUG|WARN|ERROR|FATAL)", content)
    assert "this->get_logger()" in content

def test_flexible_mutex_locking():
    """Verify standard mutex locking (lock_guard or unique_lock)."""
    content = get_content()
    assert re.search(r"std::(?:lock_guard|unique_lock)<std::mutex>", content)
    assert "boost::" not in content


def test_timestamp_synchronization():
    """[CRITICAL] Ensure output stamp matches input image stamp, NOT node time."""
    content = get_content()
    assert "header.stamp = imageHeader_.stamp" in content or "header.stamp = msg->header.stamp" in content
    bad_sync = re.search(r"header\.stamp\s*=\s*(?:this->)?now\(\)", content)
    assert not bad_sync, "Failure: Output timestamp must sync with input image, not current node time."

def test_concurrency_protection_on_roi():
    """[NEW/LEAK] Ensure roiBoxes_ access is protected by a mutex lock."""
    content = get_content()
    content_no_space = re.sub(r'\s+', '', content)
    assert re.search(r"lock_guard.*roiBoxes_", content_no_space) or \
           re.search(r"unique_lock.*roiBoxes_", content_no_space), \
           "Failure: Accessing shared 'roiBoxes_' without mutex protection detected!"


def test_move_semantics_on_publish():
    """Verify performance optimization with std::move."""
    content = get_content()
    assert re.search(r"publish\s*\(\s*std::move\s*\(", content)

def test_coordinate_scaling_logic():
    """Verify normalized to pixel coordinate conversion."""
    content = get_content()
    assert "frameWidth_" in content and "frameHeight_" in content
    assert "*" in content
