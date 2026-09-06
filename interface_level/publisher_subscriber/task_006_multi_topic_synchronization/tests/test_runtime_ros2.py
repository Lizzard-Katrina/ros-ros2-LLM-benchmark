"""
Runtime test for task_006_multi_topic_synchronization.
Launches the stereo_sync node, publishes synchronized Image messages
on /left/image and /right/image, and verifies the node processes them
by checking log output.
"""
import subprocess
import signal
import time
import os
import sys
import threading
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _read_output(proc, collected):
    """Read process output in a background thread to avoid blocking."""
    try:
        while True:
            line = proc.stdout.readline()
            if not line and proc.poll() is not None:
                break
            if line:
                collected.append(line)
    except Exception:
        pass


def test_stereo_sync_receives_synchronized_messages(rclpy_init):
    """
    Launch the stereo_sync node, publish Image messages on both topics,
    and verify the node logs the synchronized callback output.
    """
    env = os.environ.copy()
    env["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}]: {message}"

    # Launch the stereo_sync executable
    proc = subprocess.Popen(
        ["ros2", "run", "task_006_multi_topic_synchronization", "stereo_sync"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        bufsize=1,
    )

    collected_lines = []
    reader_thread = threading.Thread(target=_read_output, args=(proc, collected_lines), daemon=True)
    reader_thread.start()

    try:
        # Create a test node to publish images
        test_node = rclpy.create_node("test_stereo_publisher")

        # Use default (reliable) QoS to match the subscriber default QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        left_pub = test_node.create_publisher(Image, "/left/image", qos)
        right_pub = test_node.create_publisher(Image, "/right/image", qos)

        # Give the stereo_sync node time to start and set up subscriptions
        time.sleep(3.0)

        # Publish synchronized image pairs multiple times
        now_sec = 12345
        for i in range(40):
            left_msg = Image()
            left_msg.header.stamp = Time(sec=now_sec + i, nanosec=0)
            left_msg.header.frame_id = "left_camera"
            left_msg.height = 1
            left_msg.width = 1
            left_msg.encoding = "rgb8"
            left_msg.step = 3
            left_msg.data = bytes(3)

            right_msg = Image()
            right_msg.header.stamp = Time(sec=now_sec + i, nanosec=0)
            right_msg.header.frame_id = "right_camera"
            right_msg.height = 1
            right_msg.width = 1
            right_msg.encoding = "rgb8"
            right_msg.step = 3
            right_msg.data = bytes(3)

            left_pub.publish(left_msg)
            right_pub.publish(right_msg)

            # Spin the test node briefly
            rclpy.spin_once(test_node, timeout_sec=0.01)
            time.sleep(0.05)

        # Give the node a moment to process and flush output
        time.sleep(2.0)

        # Terminate the process gracefully
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)

        # Wait for reader thread to finish
        reader_thread.join(timeout=3)

        stdout = "".join(collected_lines)

        # Check that the syncCallback was invoked
        assert "Left stamp:" in stdout and "Right stamp:" in stdout, (
            f"syncCallback was not triggered. Node output:\n{stdout}"
        )

        # Verify the stamps match what we published
        assert str(now_sec) in stdout, (
            f"Expected stamp {now_sec} in output but got:\n{stdout}"
        )

        print(f"stereo_sync node received synchronized messages. Output:\n{stdout}")

    finally:
        # Cleanup
        try:
            test_node.destroy_node()
        except Exception:
            pass
        if proc.poll() is None:
            proc.kill()
            try:
                proc.wait(timeout=5)
            except Exception:
                pass


def test_stereo_sync_source_file_exists():
    """Verify the source file exists at the expected location."""
    from pathlib import Path
    cpp_file = Path(__file__).resolve().parent / "stereo_sync.cpp"
    assert cpp_file.exists(), f"stereo_sync.cpp not found at {cpp_file}"

    content = cpp_file.read_text()
    # Basic sanity: must have ROS2 headers
    assert "#include <rclcpp/rclcpp.hpp>" in content
    assert "class StereoSync" in content
    assert "syncCallback" in content
    assert "message_filters::Synchronizer" in content
    assert "ApproximateTime" in content
    assert "rclcpp::init" in content
    assert "rclcpp::spin" in content
    # Must NOT have ROS1 remnants
    assert "#include <ros/ros.h>" not in content
    assert "ros::init" not in content
    assert "ros::NodeHandle" not in content
    assert "boost::shared_ptr" not in content
    assert "ROS_INFO" not in content