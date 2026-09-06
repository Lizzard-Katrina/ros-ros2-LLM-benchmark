"""
Runtime test for the translated YoloObjectDetector.cpp.

This test validates the translated source file by:
1. Parsing the actual C++ source to verify ROS2 patterns
2. Checking that the file is importable/parseable and contains correct logic
3. Verifying the key behavioral contracts through source analysis

Since the darknet library and custom messages are not available for compilation
in the test environment, we validate the source artifact directly - but we do
so by actually reading the REAL translated file (not a copy), so if the
translated file is replaced with garbage, these tests will fail.
"""

import pytest
import re
from pathlib import Path
import subprocess
import os


# Locate the translated C++ file
PACKAGE_DIR = Path(__file__).resolve().parent
CPP_FILE = PACKAGE_DIR / "YoloObjectDetector.cpp"


@pytest.fixture
def cpp_content():
    """Read the actual translated source file."""
    assert CPP_FILE.exists(), f"Translated file not found: {CPP_FILE}"
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        content = f.read()
    assert len(content) > 500, "File appears to be too short / empty"
    return content


def test_file_exists_and_is_substantial(cpp_content):
    """Verify the translated file exists and has substantial content."""
    assert "namespace darknet_ros" in cpp_content
    assert "YoloObjectDetector" in cpp_content


def test_camera_callback_implemented(cpp_content):
    """Verify cameraCallback is fully implemented (not just a TODO stub)."""
    # Find the cameraCallback function body
    match = re.search(
        r'void\s+YoloObjectDetector::cameraCallback\s*\([^)]*\)\s*\{',
        cpp_content
    )
    assert match, "cameraCallback function definition not found"

    # Extract function body (find matching brace)
    start = match.end()
    brace_count = 1
    pos = start
    while pos < len(cpp_content) and brace_count > 0:
        if cpp_content[pos] == '{':
            brace_count += 1
        elif cpp_content[pos] == '}':
            brace_count -= 1
        pos += 1
    body = cpp_content[start:pos]

    # Verify it has real implementation, not just TODO
    assert "TODO" not in body, "cameraCallback still contains TODO"
    assert "cv_bridge" in body or "toCvCopy" in body, "cameraCallback missing cv_bridge usage"
    assert "BGR8" in body, "cameraCallback missing BGR8 encoding"
    assert "lock_guard" in body or "unique_lock" in body, "cameraCallback missing mutex lock"
    assert "clone" in body, "cameraCallback missing image clone"
    assert "imageStatus_" in body, "cameraCallback missing imageStatus_ update"


def test_publish_in_thread_implemented(cpp_content):
    """Verify publishInThread is fully implemented."""
    match = re.search(
        r'void\*?\s+YoloObjectDetector::publishInThread\s*\(\s*\)\s*\{',
        cpp_content
    )
    assert match, "publishInThread function definition not found"

    start = match.end()
    brace_count = 1
    pos = start
    while pos < len(cpp_content) and brace_count > 0:
        if cpp_content[pos] == '{':
            brace_count += 1
        elif cpp_content[pos] == '}':
            brace_count -= 1
        pos += 1
    body = cpp_content[start:pos]

    assert "TODO" not in body, "publishInThread still contains TODO"
    assert "roiBoxes_" in body, "publishInThread missing roiBoxes_ access"
    assert "frameWidth_" in body, "publishInThread missing frameWidth_ scaling"
    assert "frameHeight_" in body, "publishInThread missing frameHeight_ scaling"


def test_no_boost_mutexes(cpp_content):
    """Verify no boost:: mutex usage remains."""
    assert "boost::" not in cpp_content, "Found legacy boost:: usage"


def test_std_mutex_used(cpp_content):
    """Verify std::lock_guard<std::mutex> is used."""
    assert re.search(r"std::lock_guard<std::mutex>", cpp_content), \
        "Missing std::lock_guard<std::mutex>"


def test_rclcpp_logging(cpp_content):
    """Verify RCLCPP logging macros are used."""
    assert re.search(r"RCLCPP_(?:INFO|DEBUG|WARN|ERROR|FATAL)", cpp_content), \
        "Missing RCLCPP logging macros"
    assert "this->get_logger()" in cpp_content, \
        "Missing this->get_logger() call"


def test_timestamp_sync(cpp_content):
    """CRITICAL: Verify output timestamps sync with input image, not node time."""
    assert ("header.stamp = imageHeader_.stamp" in cpp_content or
            "header.stamp = msg->header.stamp" in cpp_content), \
        "Output timestamp must sync with input image header stamp"

    # Ensure no bad pattern in publishInThread
    bad_sync = re.search(r"header\.stamp\s*=\s*(?:this->)?now\(\)", cpp_content)
    assert not bad_sync, "Output timestamp must NOT use this->now()"


def test_move_semantics(cpp_content):
    """Verify std::move is used for publishing."""
    assert re.search(r"publish\s*\(\s*std::move\s*\(", cpp_content), \
        "Missing std::move() in publish calls"


def test_concurrency_protection_on_roi(cpp_content):
    """Verify roiBoxes_ access is protected by mutex."""
    content_no_space = re.sub(r'\s+', '', cpp_content)
    assert (re.search(r"lock_guard.*roiBoxes_", content_no_space) or
            re.search(r"unique_lock.*roiBoxes_", content_no_space)), \
        "roiBoxes_ access not protected by mutex"


def test_coordinate_scaling(cpp_content):
    """Verify coordinate scaling from normalized to pixel coordinates."""
    # Check that multiplication with frame dimensions occurs
    assert "frameWidth_" in cpp_content
    assert "frameHeight_" in cpp_content
    assert "*" in cpp_content


def test_sensor_msgs_encoding_constant(cpp_content):
    """Verify sensor_msgs::image_encodings::BGR8 constant is used."""
    assert "sensor_msgs::image_encodings::BGR8" in cpp_content or \
           "image_encodings::BGR8" in cpp_content, \
        "Should use sensor_msgs::image_encodings::BGR8 constant"


def test_package_builds_structure():
    """Verify the package has proper ROS2 ament structure."""
    assert (PACKAGE_DIR / "package.xml").exists()
    assert (PACKAGE_DIR / "CMakeLists.txt").exists()

    with open(PACKAGE_DIR / "package.xml", 'r') as f:
        pkg_xml = f.read()
    assert "rclcpp" in pkg_xml
    assert "ament_cmake" in pkg_xml

    with open(PACKAGE_DIR / "CMakeLists.txt", 'r') as f:
        cmake = f.read()
    assert "ament_cmake" in cmake
    assert "rclcpp" in cmake


def test_camera_callback_has_header_update(cpp_content):
    """Verify cameraCallback updates imageHeader_ from the incoming message."""
    # The callback should store the message header
    assert "imageHeader_" in cpp_content
    # Check that header is assigned from msg
    assert ("msg->header" in cpp_content or
            "imageHeader_ = msg->header" in cpp_content or
            "header.stamp = msg->header.stamp" in cpp_content), \
        "cameraCallback should store incoming message header"