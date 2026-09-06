# Copyright 2024, test suite for task_006_image_pipeline
import re
import pytest
from pathlib import Path
import subprocess
import time

# ---- Static / Oracle Tests (source code inspection) ----

# The cpp file is at the package root
CPP_FILE = Path(__file__).resolve().parent / "point_cloud_xyzrgb_radial.cpp"


def get_content():
    with open(CPP_FILE, 'r', encoding='utf-8') as f:
        return f.read()


def test_camera_info_scaling_logic():
    """Verify focal length and principal point scaling."""
    content = get_content()
    assert "k[0] *= ratio" in content or "k[0] * ratio" in content
    assert "k[2] *= ratio" in content or "k[2] * ratio" in content
    assert "k[8] *= ratio" not in content, "k[8] (constant 1.0) should not be scaled."


def test_cv_bridge_exception_safety():
    """Ensure image conversion is wrapped in try-catch for ROS 2 stability."""
    content = get_content()
    pattern = r"try\s*\{[\s\S]*cv_bridge::toCv(?:Copy|Share)[\s\S]*\}\s*catch\s*\(\s*cv_bridge::Exception"
    assert re.search(pattern, content), "Missing try-catch block for cv_bridge conversion."


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
    assert re.search(r"publish\s*\(\s*std::move\s*\(", content), \
        "Should use std::move to publish the unique_ptr."


def test_rgb_offset_style_compliance():
    """Verify the model used the required variable names for color offsets."""
    content = get_content()
    assert "red_offset =" in content
    assert "green_offset =" in content
    assert "blue_offset =" in content


def test_temporal_accuracy():
    """Ensure the pointcloud uses the message timestamp."""
    content = get_content()
    pattern = r"cloud_msg->header\.stamp\s*=\s*depth_msg->header\.stamp"
    assert re.search(pattern, content) or "cloud_msg->header = depth_msg->header" in content


def test_radial_transform_update_check():
    """Ensure radial transform is only updated when necessary."""
    content = get_content()
    assert "if" in content and "transform_" in content
    assert "width_" in content or "height_" in content, \
        "Should check for dimension changes before re-init."


def test_no_legacy_ros1_api():
    """Ensure absence of ROS 1 legacy namespaces and patterns."""
    content = get_content()
    legacy_symbols = ["ros::Time", "ros::NodeHandle", "ros::Publisher", ".toSec()", "boost::bind"]
    for symbol in legacy_symbols:
        assert symbol not in content, f"Legacy ROS 1 symbol '{symbol}' detected."


def test_color_step_variable():
    """Verify color_step variable is present."""
    content = get_content()
    assert "color_step" in content


def test_cloud_msg_unique_ptr():
    """Verify cloud_msg is a unique_ptr."""
    content = get_content()
    assert "std::make_unique<PointCloud2>" in content or \
           "std::unique_ptr<PointCloud2>" in content


def test_runtime_node_launches():
    """
    Runtime test: verify the compiled node executable can start and
    creates the expected 'points' topic.
    """
    proc = None
    try:
        # Launch the node executable
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_006_image_pipeline', 'point_cloud_xyzrgb_radial_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(3.0)

        # Check that the process is still running (didn't crash immediately)
        assert proc.poll() is None, "Node process terminated unexpectedly"

        # Check topics - the node should advertise 'points'
        topic_result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=10
        )
        assert 'points' in topic_result.stdout, \
            f"Expected 'points' topic, got: {topic_result.stdout}"

    except FileNotFoundError:
        pytest.skip("ROS 2 CLI tools not available")
    except subprocess.TimeoutExpired:
        pytest.skip("ROS 2 command timed out")
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def test_runtime_node_has_parameters():
    """
    Runtime test: verify the node declares the expected parameters.
    """
    proc = None
    try:
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_006_image_pipeline', 'point_cloud_xyzrgb_radial_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(3.0)

        assert proc.poll() is None, "Node process terminated unexpectedly"

        # Get the node name
        node_result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=10
        )
        # Find our node
        node_name = None
        for line in node_result.stdout.strip().split('\n'):
            if 'PointCloudXyzrgbRadialNode' in line or 'point_cloud' in line.lower():
                node_name = line.strip()
                break

        if node_name is None:
            # Try to use the default name
            node_name = '/PointCloudXyzrgbRadialNode'

        # Check parameters
        param_result = subprocess.run(
            ['ros2', 'param', 'list', node_name],
            capture_output=True, text=True, timeout=10
        )

        assert 'queue_size' in param_result.stdout, \
            f"Expected 'queue_size' parameter, got: {param_result.stdout}"

    except FileNotFoundError:
        pytest.skip("ROS 2 CLI tools not available")
    except subprocess.TimeoutExpired:
        pytest.skip("ROS 2 command timed out")
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()