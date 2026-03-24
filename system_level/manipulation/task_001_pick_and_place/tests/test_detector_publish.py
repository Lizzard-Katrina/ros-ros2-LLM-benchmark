import pytest
from unittest.mock import Mock

from ros2_code.object_detector_publish import VisionObjectDetector


def test_publish_detected_objects_creates_and_publishes_message():
    """
    Unit test for publish_detected_objects TODO

    We verify:
    - a message is published
    - message has detected_objects field
    """

    detector = VisionObjectDetector.__new__(VisionObjectDetector)

    detector.detected_objects_pub = Mock()
    detector.blocks_on_workbench = [Mock(), Mock()]

    detector.publish_detected_objects()

    detector.detected_objects_pub.publish.assert_called_once()

    args, _ = detector.detected_objects_pub.publish.call_args
    msg = args[0]

    assert hasattr(msg, "detected_objects")
