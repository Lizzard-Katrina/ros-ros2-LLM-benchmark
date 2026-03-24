import re
from pathlib import Path

# Direct path definitions
PUBLISHER_PATH = Path(__file__).resolve().parents[1] / "camera_publisher.py"
SUBSCRIBER_PATH = Path(__file__).resolve().parents[1] /"camera_subscriber.py"

def test_camera_publisher_translation():
    """
    Validation for Image Transport Publisher migration.
    """
    assert PUBLISHER_PATH.exists()
    content = PUBLISHER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Basic Node initialization
    assert "rclpy" in content and "Node" in content, "Oracle 1 Failed: ROS2 Node structure not found"

    # Oracle 2: Image Transport Usage
    # Since you asked for image_transport, we check for the library or the advertise call
    # Common patterns: it.advertise(...), it.create_publisher(...), or ImageTransport
    it_keywords = ["image_transport", "advertise", "CameraPublisher"]
    assert any(k in content for k in it_keywords), "Oracle 2 Failed: Image transport logic not detected"

    # Oracle 3: Image Data Handling
    # Ensure the code references Image messages or the data field
    assert "Image" in content, "Oracle 3 Failed: Image message type reference missing"

    # Oracle 4: Rate/Loop Migration
    assert any(k in content for k in ["rate", "timer", "sleep", "spin"]), "Oracle 4 Failed: Loop/Rate logic missing"


def test_camera_subscriber_translation():
    """
    Validation for Image Transport Subscriber migration.
    """
    assert SUBSCRIBER_PATH.exists()
    content = SUBSCRIBER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Basic Node initialization
    assert "rclpy" in content and "Node" in content, "Oracle 1 Failed: ROS2 Node structure not found"

    # Oracle 2: Subscription Logic
    # image_transport subscribers usually use .subscribe() instead of .create_subscription()
    sub_keywords = ["subscribe", "image_transport", "create_subscription"]
    assert any(k in content for k in sub_keywords), "Oracle 2 Failed: Subscription logic not detected"

    # Oracle 3: Callback Connection
    assert "callback" in content.lower(), "Oracle 3 Failed: Reference to callback function missing"

    # Oracle 4: ROS2 Spin
    assert "spin" in content, "Oracle 4 Failed: rclpy.spin missing"

def test_no_rospy_remnants():
    """Negative test to ensure clean migration."""
    for p in [PUBLISHER_PATH, SUBSCRIBER_PATH]:
        if p.exists():
            assert "rospy" not in p.read_text().lower(), f"ROS1 remnants found in {p.name}"
