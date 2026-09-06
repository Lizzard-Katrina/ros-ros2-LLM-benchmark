#!/usr/bin/env python3
"""
Runtime test for task_003_image_transport.

Launches the camera_publisher node via `ros2 run`, then uses a test node
to subscribe to /camera/image_raw and verify that Image messages arrive
with the expected content. Also verifies the subscriber node can be
instantiated and its callback fires.
"""
import subprocess
import sys
import time
import threading

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class ImageCollector(Node):
    """Helper node that subscribes to /camera/image_raw and stores received messages."""

    def __init__(self):
        super().__init__('test_image_collector')
        self.received_msgs = []
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._cb,
            10,
        )

    def _cb(self, msg):
        self.received_msgs.append(msg)


def test_publisher_sends_images():
    """Launch the camera_publisher and verify Image messages arrive on /camera/image_raw."""
    proc = subprocess.Popen(
        [sys.executable, '-m', 'task_003_image_transport.camera_publisher'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    collector = None
    try:
        collector = ImageCollector()
        deadline = time.time() + 8.0
        while time.time() < deadline and len(collector.received_msgs) < 3:
            rclpy.spin_once(collector, timeout_sec=0.1)

        assert len(collector.received_msgs) >= 1, (
            f"Expected at least 1 Image message, got {len(collector.received_msgs)}"
        )

        msg = collector.received_msgs[0]
        assert msg.width == 640, f"Expected width 640, got {msg.width}"
        assert msg.height == 480, f"Expected height 480, got {msg.height}"
        assert msg.encoding == 'rgb8', f"Expected encoding 'rgb8', got {msg.encoding}"
        assert len(msg.data) == 640 * 480 * 3, f"Unexpected data length: {len(msg.data)}"
    finally:
        if collector is not None:
            collector.destroy_node()
        proc.terminate()
        proc.wait(timeout=5)


def test_subscriber_node_instantiation_and_callback():
    """
    Import the CameraSubscriber class from the translated module,
    instantiate it, publish an Image, and verify the callback fires.
    """
    from task_003_image_transport.camera_subscriber import CameraSubscriber

    sub_node = CameraSubscriber()
    pub_node = Node('test_image_publisher')
    pub = pub_node.create_publisher(Image, '/camera/image_raw', 10)

    try:
        # Give DDS time to discover
        time.sleep(0.5)

        msg = Image()
        msg.width = 320
        msg.height = 240
        msg.encoding = 'bgr8'
        msg.step = 320 * 3
        msg.data = bytes(320 * 240 * 3)

        # Publish several times and spin to ensure delivery
        callback_fired = False
        original_callback = sub_node.callback

        received_data = {}

        def patched_callback(m):
            nonlocal callback_fired
            callback_fired = True
            received_data['width'] = m.width
            received_data['height'] = m.height
            received_data['encoding'] = m.encoding
            original_callback(m)

        sub_node.subscription.callback = patched_callback

        deadline = time.time() + 5.0
        while time.time() < deadline and not callback_fired:
            pub.publish(msg)
            rclpy.spin_once(sub_node, timeout_sec=0.05)
            rclpy.spin_once(pub_node, timeout_sec=0.05)

        assert callback_fired, "CameraSubscriber callback was never invoked"
        assert received_data['width'] == 320
        assert received_data['height'] == 240
        assert received_data['encoding'] == 'bgr8'
    finally:
        sub_node.destroy_node()
        pub_node.destroy_node()


def test_publisher_class_import():
    """Verify CameraPublisher can be imported and instantiated."""
    from task_003_image_transport.camera_publisher import CameraPublisher

    node = CameraPublisher()
    try:
        # Check that the publisher exists on the expected topic
        topic_names = [t[0] for t in node.get_topic_names_and_types()]
        assert '/camera/image_raw' in topic_names, (
            f"/camera/image_raw not in published topics: {topic_names}"
        )
    finally:
        node.destroy_node()