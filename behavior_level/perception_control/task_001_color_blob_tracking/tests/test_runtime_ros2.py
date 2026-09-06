"""
Runtime test for task_001_color_blob_tracking.

Launches both drive_bot and process_image nodes, publishes a synthetic
camera image with a white blob on the LEFT side, and verifies that the
resulting /cmd_vel message has a positive angular.z (turn left) and
linear.x == 0.
"""
import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


@pytest.fixture(scope="module", autouse=True)
def ros_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_test_image(width=800, height=600, blob_col=50):
    """
    Create a sensor_msgs/Image with a white pixel at (row=0, col=blob_col).
    All other pixels are black. encoding=rgb8, step=width*3.
    """
    img = Image()
    img.header.frame_id = "camera"
    img.height = height
    img.width = width
    img.encoding = "rgb8"
    img.is_bigendian = 0
    img.step = width * 3
    # All zeros (black)
    data = bytearray(height * width * 3)
    # Place white pixel at (row=0, col=blob_col)
    idx = blob_col * 3
    data[idx] = 255
    data[idx + 1] = 255
    data[idx + 2] = 255
    img.data = bytes(data)
    return img


def test_blob_left_turns_left():
    """
    Publish an image with a white blob on the LEFT third.
    Expect angular.z > 0 (turn left) on /cmd_vel.
    """
    procs = []
    test_node = None
    try:
        # Launch drive_bot
        p_drive = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "drive_bot"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_drive)
        time.sleep(2)

        # Launch process_image
        p_proc = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "process_image"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_proc)
        time.sleep(2)

        test_node = Node("test_blob_tracking")

        # Publisher for camera image
        pub = test_node.create_publisher(Image, "/camera/rgb/image_raw", 10)

        # Subscriber for cmd_vel
        received_msgs = []

        def cmd_vel_cb(msg):
            received_msgs.append(msg)

        sub = test_node.create_subscription(Twist, "/cmd_vel", cmd_vel_cb, 10)

        # Create image with blob on the left side (col=50 out of 800 → left third)
        img = make_test_image(width=800, height=600, blob_col=50)

        # Publish several times and spin to allow the pipeline to process
        deadline = time.time() + 10.0
        while time.time() < deadline and len(received_msgs) == 0:
            pub.publish(img)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No /cmd_vel message received within timeout"

        last = received_msgs[-1]
        # Blob is on the left → angular.z should be positive (turn left), linear.x ~ 0
        assert last.angular.z > 0, \
            f"Expected positive angular.z for left blob, got {last.angular.z}"
        assert abs(last.linear.x) < 0.01, \
            f"Expected ~0 linear.x for left blob, got {last.linear.x}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        for p in procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


def test_blob_center_goes_straight():
    """
    Publish an image with a white blob in the CENTER third.
    Expect linear.x > 0 and angular.z == 0 on /cmd_vel.
    """
    procs = []
    test_node = None
    try:
        p_drive = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "drive_bot"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_drive)
        time.sleep(2)

        p_proc = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "process_image"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_proc)
        time.sleep(2)

        test_node = Node("test_blob_center")

        pub = test_node.create_publisher(Image, "/camera/rgb/image_raw", 10)

        received_msgs = []

        def cmd_vel_cb(msg):
            received_msgs.append(msg)

        sub = test_node.create_subscription(Twist, "/cmd_vel", cmd_vel_cb, 10)

        # Blob in center (col=400 out of 800)
        img = make_test_image(width=800, height=600, blob_col=400)

        deadline = time.time() + 10.0
        while time.time() < deadline and len(received_msgs) == 0:
            pub.publish(img)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No /cmd_vel message received within timeout"

        last = received_msgs[-1]
        assert last.linear.x > 0, \
            f"Expected positive linear.x for center blob, got {last.linear.x}"
        assert abs(last.angular.z) < 0.01, \
            f"Expected ~0 angular.z for center blob, got {last.angular.z}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        for p in procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


def test_no_blob_stops():
    """
    Publish an all-black image (no white blob).
    Expect both linear.x == 0 and angular.z == 0 on /cmd_vel.
    """
    procs = []
    test_node = None
    try:
        p_drive = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "drive_bot"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_drive)
        time.sleep(2)

        p_proc = subprocess.Popen(
            ["ros2", "run", "task_001_color_blob_tracking", "process_image"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p_proc)
        time.sleep(2)

        test_node = Node("test_no_blob")

        pub = test_node.create_publisher(Image, "/camera/rgb/image_raw", 10)

        received_msgs = []

        def cmd_vel_cb(msg):
            received_msgs.append(msg)

        sub = test_node.create_subscription(Twist, "/cmd_vel", cmd_vel_cb, 10)

        # All-black image (no white blob at all)
        img = Image()
        img.header.frame_id = "camera"
        img.height = 600
        img.width = 800
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = 800 * 3
        img.data = bytes(600 * 800 * 3)  # all zeros

        deadline = time.time() + 10.0
        while time.time() < deadline and len(received_msgs) == 0:
            pub.publish(img)
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No /cmd_vel message received within timeout"

        last = received_msgs[-1]
        assert abs(last.linear.x) < 0.01, \
            f"Expected ~0 linear.x for no blob, got {last.linear.x}"
        assert abs(last.angular.z) < 0.01, \
            f"Expected ~0 angular.z for no blob, got {last.angular.z}"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        for p in procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()