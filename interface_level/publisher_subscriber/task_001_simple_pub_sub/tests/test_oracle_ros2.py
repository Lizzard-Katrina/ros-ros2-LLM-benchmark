import re
from pathlib import Path

LISTENER_PATH = Path(__file__).resolve().parents[1] / "listener.py"
TALKER_PATH   = Path(__file__).resolve().parents[1] / "talker.py"

def test_listener_ros2_translation():
    """
    Oracle test tailored for the 'Fill-in-the-blank' migration task.
    Validates node initialization, subscription, and spin logic.
    """
    assert LISTENER_PATH.exists(), f"Missing file: {LISTENER_PATH}"
    content = LISTENER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Node Initialization
    # Since the TODO was inside a function, we check for Node creation.
    # Matches: node = Node('listener') or self.node = rclpy.create_node('listener')
    node_init = r"(?:Node|create_node)\s*\(\s*['\"]listener['\"]\s*\)"
    assert re.search(node_init, content), \
        "Oracle 1 Failed: Must initialize ROS2 node with name 'listener'"

    # Oracle 2: Subscription Logic
    # Checks for: .create_subscription(String, 'chatter', ...)
    # We use re.DOTALL to handle potential multi-line arguments
    sub_pattern = r"create_subscription\s*\(\s*String\s*,\s*['\"]chatter['\"]\s*,"
    assert re.search(sub_pattern, content, re.DOTALL), \
        "Oracle 2 Failed: Must subscribe to topic 'chatter' with String type"

    # Oracle 3: Callback Connection
    # Ensures the subscription actually points to the 'callback' function defined earlier
    callback_pattern = r"create_subscription\s*\(.*,\s*callback[\s,)]"
    assert re.search(callback_pattern, content, re.DOTALL), \
        "Oracle 3 Failed: Subscription must use the defined 'callback' function"

    # Oracle 4: Keep Spin
    # Matches: rclpy.spin(node) or rclpy.spin(self)
    spin_pattern = r"rclpy\.spin\s*\(\s*\w+\s*\)"
    assert re.search(spin_pattern, content), \
        "Oracle 4 Failed: Missing rclpy.spin() to keep the node alive"

    # Oracle 5: Clean Migration (Negative Test)
    # Ensure no ROS1 'rospy' remains in the logic
    assert "rospy" not in content.lower(), \
        "Oracle 5 Failed: Code contains ROS1 'rospy' references"

def test_talker_ros2_translation():
    assert TALKER_PATH.exists(), f"Missing file: {TALKER_PATH}"
    content = TALKER_PATH.read_text(encoding="utf-8")
    
    # Oracle 1: Node subclass
    node_pattern = r"class\s+\w+\s*\(\s*(?:[\w\.]+\.)?Node[\s,]*\)"
    assert re.search(node_pattern, content), "Oracle 1 Failed: Talker must define a ROS2 Node subclass"
    
    # Oracle 2 & 3: Node Name
    assert re.search(r"['\"]talker['\"]", content), "Node name is not 'talker'"
    
    # Oracle 4: Publisher
    assert re.search(r"create_publisher\s*\(\s*String\s*,\s*['\"]chatter['\"]\s*,", content), "Incorrect publisher"
