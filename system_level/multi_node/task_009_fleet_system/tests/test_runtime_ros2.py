"""
Runtime test for task_009_fleet_system.
Tests the msg2fbs node by subscribing to its published schema topic
and verifying the content matches ROS 2 conventions.
"""
import subprocess
import sys
import time
import threading
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SchemaCollector(Node):
    def __init__(self):
        super().__init__('schema_collector')
        self.received_data = None
        self.sub = self.create_subscription(
            String, 'fbs_schema', self.callback, 10
        )

    def callback(self, msg):
        self.received_data = msg.data


def test_msg2fbs_node_publishes_valid_schema():
    """Launch the msg2fbs_node and verify it publishes a valid FBS schema."""
    rclpy.init()
    collector = SchemaCollector()
    proc = None

    try:
        # Launch the node as a subprocess
        proc = subprocess.Popen(
            [sys.executable, '-m', 'task_009_fleet_system.msg2fbs_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Spin until we get data or timeout
        deadline = time.time() + 10.0
        while time.time() < deadline and collector.received_data is None:
            rclpy.spin_once(collector, timeout_sec=0.1)

        schema = collector.received_data
        assert schema is not None, "No schema received from msg2fbs_node within timeout"

        # Verify ROS 2 naming conventions
        assert "sec:uint32" in schema, "Schema must contain 'sec:uint32'"
        assert "nanosec:uint32" in schema, "Schema must contain 'nanosec:uint32'"

        # Verify struct (not table) for RosTime
        assert "struct RosTime" in schema, "RosTime must be a struct"
        assert "struct RosDuration" in schema, "RosDuration must be a struct"

        # Verify no ROS 1 naming leakage
        import re
        assert not re.search(r"\bsecs\b", schema), "ROS 1 'secs' found in schema"
        assert not re.search(r"\bnsecs\b", schema), "ROS 1 'nsecs' found in schema"

        # Verify metadata
        assert "MsgMetadata" in schema, "MsgMetadata must be present"
        assert "__metadata" in schema, "__metadata field must be present"

        # Verify MsgWithMetadata
        assert "MsgWithMetadata" in schema, "MsgWithMetadata must be present"

    finally:
        collector.destroy_node()
        rclpy.shutdown()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=5)


def test_type_class_namespace_parsing():
    """Test the Type class directly for namespace parsing logic."""
    import os
    import sys
    pkg_root = os.path.dirname(os.path.abspath(__file__))
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    import msg2fbs
    msg2fbs.BASE_NS = "fb"

    # Test ROS 2 style: std_msgs/msg/Header
    t = msg2fbs.Type("std_msgs/msg/Header")
    assert t.name == "Header"
    assert t.namespace == "std_msgs.msg"
    assert t.is_array is False
    assert t.ros_type == "std_msgs/msg/Header"

    # Test simple type
    t2 = msg2fbs.Type("float32")
    assert t2.name == "float32"
    assert t2.namespace is None
    assert t2.is_array is False

    # Test array type
    t3 = msg2fbs.Type("float32[]")
    assert t3.name == "float32"
    assert t3.is_array is True
    assert t3.fbs_type() == "[float32]"

    # Test ROS 1 style: std_msgs/Header
    t4 = msg2fbs.Type("std_msgs/Header")
    assert t4.name == "Header"
    assert t4.namespace == "std_msgs"

    # Test fixed-size array
    t5 = msg2fbs.Type("float64[36]")
    assert t5.name == "float64"
    assert t5.is_array is True


def test_gen_support_output():
    """Test gen_support() produces correct ROS 2 output."""
    import os
    import sys
    pkg_root = os.path.dirname(os.path.abspath(__file__))
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    import msg2fbs
    msg2fbs.BASE_NS = "fb"

    lines = list(msg2fbs.gen_support())
    combined = "\n".join(lines)

    # Must have struct RosTime with sec/nanosec
    assert "struct RosTime {" in combined
    assert "sec:uint32;" in combined
    assert "nanosec:uint32;" in combined

    # Must have struct RosDuration with sec/nanosec
    assert "struct RosDuration {" in combined
    assert "sec:int32;" in combined
    assert "nanosec:int32;" in combined

    # Must have MsgMetadata
    assert "table MsgMetadata {" in combined

    # Must have MsgWithMetadata
    assert "table MsgWithMetadata {" in combined

    # No ROS 1 naming
    import re
    assert not re.search(r"\bsecs\b", combined)
    assert not re.search(r"\bnsecs\b", combined)


def test_fbs_required_attribute():
    """Verify non-scalar fields have (required) attribute in schema.fbs."""
    from pathlib import Path
    fbs_file = Path(__file__).resolve().parent / "schema.fbs"
    content = fbs_file.read_text()

    # stamp is RosTime (non-scalar in table context) -> must be required
    assert "stamp:RosTime (required)" in content or "stamp:fb.RosTime (required)" in content, \
        "stamp field must have (required) attribute"

    # frame_id is string (non-scalar) -> must be required
    assert "frame_id:string (required)" in content, \
        "frame_id field must have (required) attribute"