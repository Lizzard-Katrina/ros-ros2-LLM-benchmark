"""
Runtime test for task_002_camera_edge_detection.
Publishes a synthetic BGR image on camera/image_raw, then checks that
the edge_detection_node publishes a mono8 edge image on camera/edges.
"""
import subprocess
import sys
import time
import numpy as np
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_edge_detection_pipeline(ros_context):
    """Launch the real edge_detection_node, publish an image, and verify output."""
    # Launch the actual installed node via ros2 run
    proc = subprocess.Popen(
        [sys.executable, "-c",
         "from task_002_camera_edge_detection.camera_edge import main; main()"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    received_msgs = []

    class TestHelper(Node):
        def __init__(self):
            super().__init__('test_edge_helper')
            self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
            self.subscription = self.create_subscription(
                Image, 'camera/edges', self._cb, 10)
            self.bridge = CvBridge()

        def _cb(self, msg):
            received_msgs.append(msg)

        def publish_test_image(self):
            # Create a 64x64 BGR image with a white rectangle on black background
            # This guarantees strong edges for Canny to detect
            img = np.zeros((64, 64, 3), dtype=np.uint8)
            img[16:48, 16:48, :] = 255  # white square
            msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'test_camera_frame'
            self.publisher.publish(msg)

    helper = TestHelper()

    try:
        # Give the node time to start
        time.sleep(1.0)

        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            helper.publish_test_image()
            rclpy.spin_once(helper, timeout_sec=0.1)
            if received_msgs:
                break

        assert len(received_msgs) > 0, "No edge image received on 'camera/edges' topic."

        edge_msg = received_msgs[0]

        # Check encoding is mono8
        assert edge_msg.encoding == 'mono8', \
            f"Expected encoding 'mono8', got '{edge_msg.encoding}'"

        # Check header was copied (frame_id should match)
        assert edge_msg.header.frame_id == 'test_camera_frame', \
            f"Header frame_id mismatch: expected 'test_camera_frame', got '{edge_msg.header.frame_id}'"

        # Check dimensions match input
        assert edge_msg.height == 64, f"Expected height 64, got {edge_msg.height}"
        assert edge_msg.width == 64, f"Expected width 64, got {edge_msg.width}"

        # Decode and verify there are actual edge pixels (non-zero)
        bridge = CvBridge()
        edge_image = bridge.imgmsg_to_cv2(edge_msg, 'mono8')
        nonzero_count = np.count_nonzero(edge_image)
        assert nonzero_count > 0, "Edge image has no detected edges (all zeros)."

    finally:
        helper.destroy_node()
        proc.terminate()
        proc.wait(timeout=5)