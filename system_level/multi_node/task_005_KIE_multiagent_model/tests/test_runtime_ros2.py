"""
Runtime test for the ROS 2 migrated fkie_mas_discovery components.
Tests both interface_finder.py and master_discovery.py.
"""
import os
import sys
import time
import threading
import struct
import pytest

# Ensure the package root is on the path so top-level shims and package are importable
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)


def test_interface_finder_get_topic_names_and_types():
    """
    Test that _get_topic uses get_topic_names_and_types from the ROS 2 graph API.
    Creates a publisher on a known topic, then verifies _get_topic can find it.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    try:
        # Create a node that publishes on a topic with a known type
        pub_node = Node('test_publisher_node')
        pub = pub_node.create_publisher(String, '/test_discovery_topic', 10)

        # Create a finder node
        finder_node = Node('test_finder_node')

        # Give DDS time to discover
        time.sleep(1.0)

        # Spin briefly to allow discovery
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(finder_node)

        end_time = time.time() + 3.0
        while time.time() < end_time:
            executor.spin_once(timeout_sec=0.1)

        # Now use the graph API directly (same as interface_finder does)
        topic_names_and_types = finder_node.get_topic_names_and_types()

        # Verify we can find our topic
        found_topics = []
        for topic_name, topic_types in topic_names_and_types:
            for topic_type in topic_types:
                if 'String' in topic_type:
                    found_topics.append(topic_name)

        assert '/test_discovery_topic' in found_topics, \
            f"Expected '/test_discovery_topic' in {found_topics}"

        # Also test the _get_topic_from_node function from the package
        from task_005_KIE_multiagent_model.interface_finder import _get_topic_from_node
        result = _get_topic_from_node(finder_node, 'String', wait=False, check_host=False)
        assert isinstance(result, list), "Result should be a list"
        assert '/test_discovery_topic' in result, \
            f"Expected '/test_discovery_topic' in {result}"

    finally:
        pub_node.destroy_node()
        finder_node.destroy_node()
        rclpy.shutdown()


def test_interface_finder_no_xmlrpc():
    """Verify interface_finder.py does not use xmlrpc."""
    finder_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'interface_finder.py')
    if not os.path.exists(finder_path):
        finder_path = os.path.join(PKG_DIR, 'interface_finder.py')
    with open(finder_path, 'r') as f:
        code = f.read()
    assert 'xmlrpc' not in code
    assert 'ServerProxy' not in code


def test_interface_finder_uses_graph_api():
    """Verify interface_finder.py uses get_topic_names_and_types."""
    finder_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'interface_finder.py')
    if not os.path.exists(finder_path):
        finder_path = os.path.join(PKG_DIR, 'interface_finder.py')
    with open(finder_path, 'r') as f:
        code = f.read()
    assert 'get_topic_names_and_types' in code


def test_interface_finder_has_get_hostname():
    """Verify interface_finder.py uses get_hostname for host filtering."""
    finder_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'interface_finder.py')
    if not os.path.exists(finder_path):
        finder_path = os.path.join(PKG_DIR, 'interface_finder.py')
    with open(finder_path, 'r') as f:
        code = f.read()
    assert 'get_hostname' in code
    assert 'own_host' in code


def test_master_discovery_publishers():
    """Verify master_discovery.py uses create_publisher with correct types."""
    md_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'master_discovery.py')
    if not os.path.exists(md_path):
        md_path = os.path.join(PKG_DIR, 'master_discovery.py')
    with open(md_path, 'r') as f:
        code = f.read()
    assert 'create_publisher' in code
    assert 'MasterState' in code
    assert 'LinkStatesStamped' in code
    assert 'from rclpy' in code


def test_master_discovery_no_rospy_time():
    """Verify master_discovery.py does not use rospy.Time."""
    md_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'master_discovery.py')
    if not os.path.exists(md_path):
        md_path = os.path.join(PKG_DIR, 'master_discovery.py')
    with open(md_path, 'r') as f:
        code = f.read()
    assert 'rospy.Time' not in code


def test_master_discovery_struct_pack():
    """Verify master_discovery.py preserves the UDP protocol with struct.pack."""
    md_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'master_discovery.py')
    if not os.path.exists(md_path):
        md_path = os.path.join(PKG_DIR, 'master_discovery.py')
    with open(md_path, 'r') as f:
        code = f.read()
    assert 'struct.pack' in code
    assert 'Discoverer.HEARTBEAT_FMT' in code


def test_master_discovery_pubchanges_consistency():
    """Verify self.pubchanges is defined and used to publish."""
    md_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'master_discovery.py')
    if not os.path.exists(md_path):
        md_path = os.path.join(PKG_DIR, 'master_discovery.py')
    with open(md_path, 'r') as f:
        code = f.read()
    import re
    assert re.search(r'self\.pubchanges\s*=', code)
    assert re.search(r'self\.pubchanges\.publish\(', code)


def test_heartbeat_format_struct():
    """Test that the HEARTBEAT_FMT produces valid binary messages."""
    fmt = 'cBBiiHii'
    msg = struct.pack(fmt, b'R', 2, 2, 100, 200, 11611, 100, 200)
    assert len(msg) == struct.calcsize(fmt)
    unpacked = struct.unpack(fmt, msg)
    assert unpacked[0] == b'R'
    assert unpacked[1] == 2
    assert unpacked[3] == 100
    assert unpacked[5] == 11611


def test_master_discovery_msg2masterState():
    """Test the msg2masterState class method."""
    from task_005_KIE_multiagent_model.master_discovery import Discoverer

    fmt = Discoverer.HEARTBEAT_FMT
    msg = struct.pack(fmt, b'R', Discoverer.VERSION,
                      int(0.02 * 10), 1000, 500,
                      11611, 1000, 500)
    version, values = Discoverer.msg2masterState(msg, ('127.0.0.1', 11611))
    assert version == Discoverer.VERSION
    assert values[0] == b'R'
    assert values[3] == 1000
    assert values[5] == 11611


def test_interface_finder_return_list():
    """Verify _get_topic returns a list."""
    finder_path = os.path.join(PKG_DIR, 'task_005_KIE_multiagent_model', 'interface_finder.py')
    if not os.path.exists(finder_path):
        finder_path = os.path.join(PKG_DIR, 'interface_finder.py')
    with open(finder_path, 'r') as f:
        code = f.read()
    import re
    assert re.search(r'result\s*=\s*\[\]', code)
    assert re.search(r'return\s+result', code)