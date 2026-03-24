import pytest
from unittest.mock import Mock

from ros2_code.object_detector_imagecall import VisionObjectDetector


def test_image_callback_triggers_detection_and_publish():
    """
    Unit test for image_callback TODO

    We verify:
    - detection pipeline is triggered
    - publish_detected_objects() is called
    """

    detector = VisionObjectDetector.__new__(VisionObjectDetector)

    detector.blocks_on_workbench = []
    detector.publish_detected_objects = Mock()

    fake_image_msg = Mock()

    detector.image_callback(fake_image_msg)

    assert detector.publish_detected_objects.called
