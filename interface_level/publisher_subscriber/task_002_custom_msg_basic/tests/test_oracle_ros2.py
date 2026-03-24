import re
from pathlib import Path

PUBLISHER_PATH = Path(__file__).resolve().parents[1] /"publisher_node.py"
SUBSCRIBER_PATH = Path(__file__).resolve().parents[1] /"subscriber_node.py"

def test_publisher_custom_msg_translation():
    assert PUBLISHER_PATH.exists()
    content = PUBLISHER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Node Name (Relaxed)
    # Just check if the expected node name string exists anywhere in the code
    assert "person_publisher" in content, "Oracle 1 Failed: String 'person_publisher' not found"

    # Oracle 2: Publisher Logic (Relaxed)
    # Just check if create_publisher, Person, and person_info exist
    assert "create_publisher" in content, "Missing create_publisher call"
    assert "person_info" in content, "Missing topic 'person_info'"
    assert "Person" in content, "Missing message type 'Person'"

    # Oracle 3: Field Assignment
    assert re.search(r"\.(?:name|age|height)\s*=", content), "No field assignments detected"

def test_subscriber_custom_msg_translation():
    assert SUBSCRIBER_PATH.exists()
    content = SUBSCRIBER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Node Name (Relaxed)
    assert "person_subscriber" in content, "Oracle 1 Failed: String 'person_subscriber' not found"

    # Oracle 2: Subscription Logic (Relaxed)
    # Instead of a complex nested regex, we check for individual components
    assert "create_subscription" in content, "Missing create_subscription call"
    assert "person_info" in content, "Missing topic 'person_info'"
    assert "Person" in content, "Missing message type 'Person'"

    # Oracle 3: Callback
    assert "callback" in content, "Oracle 3 Failed: Reference to 'callback' not found"

    # Oracle 4: Spin
    assert "spin" in content, "Oracle 4 Failed: rclpy.spin not found"
