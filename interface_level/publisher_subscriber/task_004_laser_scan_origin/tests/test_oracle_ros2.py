import re
from pathlib import Path

# Direct path definitions
PUBLISHER_PATH = Path(__file__).resolve().parents[1] /"lidar_publisher.py"
SUBSCRIBER_PATH = Path(__file__).resolve().parents[1] /"lidar_subscriber.py"

############################
# Publisher Oracle (Strict)
############################

def test_lidar_publisher_translation():
    """
    Strict structural audit for Lidar Publisher.
    """
    assert PUBLISHER_PATH.exists()
    content = PUBLISHER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Proper Node Initialization
    # Must use Node('name') or create_node('name')
    assert re.search(r"(?:Node|create_node)\s*\(\s*['\"]lidar_publisher['\"]\s*\)", content), \
        "Oracle 1 Failed: Incorrect Node name or initialization"

    # Oracle 2: Strict Publisher Signature
    # Logic: .create_publisher(LaserScan, 'scan', qos_profile)
    # We allow flexible spacing/newlines but keep the parameter order strict.
    pub_pattern = r"create_publisher\s*\(\s*LaserScan\s*,\s*['\"]/?scan['\"]\s*,"
    assert re.search(pub_pattern, content, re.DOTALL), \
        "Oracle 2 Failed: Must call create_publisher with (LaserScan, '/scan', ...)"

    # Oracle 3: Field Assignment Audit
    # For a benchmark, we expect the model to at least assign to the 'ranges' and 'header'
    assert re.search(r"\.header\.stamp\s*=", content), "Oracle 3a Failed: Missing header.stamp assignment"
    assert re.search(r"\.ranges\s*=", content), "Oracle 3b Failed: Missing ranges assignment"

    # Oracle 4: ROS2 Spin Requirement
    assert "rclpy.spin" in content, "Oracle 4 Failed: Missing rclpy.spin() for the node"


############################
# Subscriber Oracle (Strict)
############################

def test_lidar_subscriber_translation():
    """
    Strict structural audit for Lidar Subscriber.
    """
    assert SUBSCRIBER_PATH.exists()
    content = SUBSCRIBER_PATH.read_text(encoding="utf-8")

    # Oracle 1: Node Initialization
    assert re.search(r"(?:Node|create_node)\s*\(\s*['\"]lidar_subscriber['\"]\s*\)", content), \
        "Oracle 1 Failed: Incorrect Node name or initialization"

    # Oracle 2: Strict Subscription Signature
    # Logic: .create_subscription(LaserScan, 'scan', callback, qos_profile)
    sub_pattern = r"create_subscription\s*\(\s*LaserScan\s*,\s*['\"]/?scan['\"]\s*,\s*callback\s*,"
    assert re.search(sub_pattern, content, re.DOTALL), \
        "Oracle 2 Failed: Must call create_subscription with (LaserScan, '/scan', callback, ...)"

    # Oracle 3: Spin with Node
    # In ROS2 functional style, spin must take the node as an argument
    assert re.search(r"rclpy\.spin\s*\(\s*\w+\s*\)", content), \
        "Oracle 3 Failed: rclpy.spin() must take the node object as an argument"

############################
# Clean Migration Test
############################

def test_no_rospy_remnants():
    """Hard fail if any rospy call remains."""
    for p in [PUBLISHER_PATH, SUBSCRIBER_PATH]:
        if p.exists():
            content = p.read_text()
            # Ensure no 'rospy.' calls are made
            assert not re.search(r"rospy\.", content), f"Oracle Failed: {p.name} still uses rospy APIs"
