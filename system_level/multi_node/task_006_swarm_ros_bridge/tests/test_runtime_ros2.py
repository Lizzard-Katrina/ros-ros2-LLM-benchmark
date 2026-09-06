"""
Runtime test for the BridgeNode.
Since the node requires ZMQ and complex configuration, we test by:
1. Verifying the source files contain correct ROS2 patterns (static checks that mirror oracle).
2. Launching the node with default (no topics) params and verifying it starts and creates the node.
3. Checking the node is discoverable and has the expected name.
"""

import subprocess
import time
import re
import pytest
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
HPP_FILE = BASE_DIR / "bridge_node.hpp"
CPP_FILE = BASE_DIR / "bridge_node.cpp"
INCLUDE_HPP = BASE_DIR / "include" / "bridge_node.hpp"
SRC_CPP = BASE_DIR / "src" / "bridge_node.cpp"


def get_hpp_code():
    """Get header code from whichever location exists."""
    for p in [HPP_FILE, INCLUDE_HPP]:
        if p.exists():
            return p.read_text()
    pytest.fail("bridge_node.hpp not found")


def get_cpp_code():
    """Get source code from whichever location exists."""
    for p in [CPP_FILE, SRC_CPP]:
        if p.exists():
            return p.read_text()
    pytest.fail("bridge_node.cpp not found")


class TestBridgeNodeSourceValidation:
    """Validate the migrated source code for ROS2 correctness."""

    def test_hpp_class_inheritance(self):
        """Verify BridgeNode inherits from rclcpp::Node."""
        hpp = get_hpp_code()
        assert re.search(r'class\s+BridgeNode\s*:\s*public\s+rclcpp::Node', hpp), \
            "BridgeNode must inherit from rclcpp::Node"

    def test_hpp_ros2_shared_ptr_interfaces(self):
        """Verify ROS2 SharedPtr interfaces in header."""
        hpp = get_hpp_code()
        pattern = r"std::vector<rclcpp::(?:Subscription|Publisher)(?:Base|Generic)?::SharedPtr>"
        assert re.search(pattern, hpp), \
            "Header must declare ROS 2 SharedPtr interfaces."

    def test_hpp_no_ros1(self):
        """No ROS1 includes in header."""
        hpp = get_hpp_code()
        assert "#include <ros/ros.h>" not in hpp
        assert "ros::Subscriber" not in hpp

    def test_cpp_create_publisher(self):
        """Check this->create_publisher or this->create_subscription usage."""
        cpp = get_cpp_code()
        pattern = r"(?:this->|node->)create_(?:publisher|subscription)"
        assert re.search(pattern, cpp), \
            "Source must use ROS 2 node creation patterns."

    def test_cpp_serialization(self):
        """Verify rclcpp::Serialization usage."""
        cpp = get_cpp_code()
        assert "rclcpp::Serialization" in cpp, \
            "Must use rclcpp::Serialization"
        assert "ros::serialization" not in cpp, \
            "Must not have legacy ros::serialization"

    def test_cpp_qos(self):
        """Verify QoS usage."""
        cpp = get_cpp_code()
        pattern = r"rclcpp::(?:QoS|SystemDefaultsQoS|SensorDataQoS)"
        assert re.search(pattern, cpp), \
            "Migration should include ROS 2 QoS profiles."

    def test_cpp_callback_signature(self):
        """Verify callback uses SharedPtr."""
        cpp = get_cpp_code()
        # Match either the template definition or explicit instantiation lines
        pattern = r"sub_cb\s*[\(<].*?const\s+[\w:]+::SharedPtr\s+\w+"
        # Also check for explicit instantiation pattern
        pattern2 = r"sub_cb\s*<[\w:]+>\s*\(const\s+[\w:]+::SharedPtr\s+\w+"
        assert re.search(pattern, cpp) or re.search(pattern2, cpp), \
            "Callback signature should use SharedPtr for ROS 2 compatibility."

    def test_cpp_logging(self):
        """Verify RCLCPP_INFO usage and no ROS_INFO."""
        cpp = get_cpp_code()
        assert "RCLCPP_INFO" in cpp
        assert "ROS_INFO" not in cpp

    def test_no_excessive_repetition(self):
        """Detect code spamming."""
        cpp = get_cpp_code()
        lines = cpp.split('\n')
        unique_lines = set(lines)
        if len(lines) > 20:
            ratio = len(unique_lines) / len(lines)
            assert ratio >= 0.5, \
                f"Detected code spamming (unique ratio: {ratio:.2f})"


class TestBridgeNodeRuntime:
    """Runtime test: launch the node and verify it's alive."""

    def test_node_launches_and_is_discoverable(self):
        """Launch bridge_node with no topics configured, verify node name appears."""
        import rclpy
        from rclpy.node import Node

        # Start the bridge node as a subprocess
        proc = None
        rclpy.init()
        test_node = None
        try:
            proc = subprocess.Popen(
                ['ros2', 'run', 'task_006_swarm_ros_bridge', 'bridge_node'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Give it time to start
            time.sleep(3.0)

            # Check it's still running (didn't crash)
            assert proc.poll() is None, \
                f"Node process exited prematurely with code {proc.returncode}"

            # Use ros2 node list to find our node
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5
            )

            node_list = result.stdout.strip()
            assert '/swarm_bridge' in node_list, \
                f"Expected /swarm_bridge in node list, got: {node_list}"

            # Also verify we can get node info
            test_node = Node('test_bridge_checker')

            # Get the list of node names
            node_names = test_node.get_node_names()
            assert 'swarm_bridge' in node_names, \
                f"Expected 'swarm_bridge' in node names: {node_names}"

        finally:
            if test_node:
                test_node.destroy_node()
            rclpy.shutdown()
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)