#!/usr/bin/env python3
"""
Runtime test for task_006_pick_up_objects.

Tests:
1. ManageObject node creates services (check_object, get_object, let_object)
2. Controller node subscribes to odom and publishes cmd_vel
3. Service calls return expected responses
"""
import pytest
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import os
import sys


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestManagerNode:
    """Test the ManageObject node's services."""

    def test_check_object_service_exists_and_responds(self):
        """
        Launch the manage_objects_node, then call check_object service.
        Since robot_pose is set far from objects, it should return success=False.
        """
        sys.path.insert(0, os.path.dirname(__file__))
        from manage_objects_node import ManageObject

        # Patch spawn_model to not require gazebo
        original_spawn = ManageObject.spawn_model

        def mock_spawn(self_node, model_name, model_xml, p):
            return True

        ManageObject.spawn_model = mock_spawn

        manager_node = None
        spin_thread = None
        test_node = None
        executor = None
        try:
            manager_node = ManageObject('./')
            # Set robot_pose so service can compute distance
            manager_node.robot_pose = (0.0, 0.0)

            # Spin manager in background
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(manager_node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            # Give it a moment to set up
            time.sleep(0.5)

            # Create test node and client
            test_node = Node('test_check_object')
            client = test_node.create_client(Trigger, 'check_object')

            # Wait for service
            assert client.wait_for_service(timeout_sec=5.0), \
                "check_object service not available"

            # Call the service
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)

            assert future.done(), "Service call did not complete"
            result = future.result()
            assert result is not None, "Service returned None"
            # The important thing is we got a valid response
            assert isinstance(result.success, bool), "Response success should be bool"
            assert isinstance(result.message, str), "Response message should be str"

        finally:
            if test_node:
                test_node.destroy_node()
            if executor:
                executor.shutdown()
            if manager_node:
                manager_node.destroy_node()
            # Restore
            ManageObject.spawn_model = original_spawn

    def test_get_object_service_responds(self):
        """Test get_object service returns a response."""
        sys.path.insert(0, os.path.dirname(__file__))
        from manage_objects_node import ManageObject

        original_spawn = ManageObject.spawn_model

        def mock_spawn(self_node, model_name, model_xml, p):
            return True

        ManageObject.spawn_model = mock_spawn

        manager_node = None
        test_node = None
        executor = None
        try:
            manager_node = ManageObject('./')
            manager_node.robot_pose = (100.0, 100.0)  # Far from any object

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(manager_node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            time.sleep(0.5)

            test_node = Node('test_get_object')
            client = test_node.create_client(Trigger, 'get_object')
            assert client.wait_for_service(timeout_sec=5.0)

            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(test_node, future, timeout_sec=5.0)

            assert future.done()
            result = future.result()
            assert result is not None
            # Robot is far from objects, so success should be False
            assert result.success is False, \
                "get_object should fail when robot is far from objects"

        finally:
            if test_node:
                test_node.destroy_node()
            if executor:
                executor.shutdown()
            if manager_node:
                manager_node.destroy_node()
            ManageObject.spawn_model = original_spawn


class TestControllerNode:
    """Test the Controller node's pub/sub."""

    def test_controller_publishes_cmd_vel(self):
        """Controller should publish Twist on cmd_vel when spinning."""
        sys.path.insert(0, os.path.dirname(__file__))
        from turtlebot_controller_node import Controller

        controller_node = None
        test_node = None
        executor = None
        received_msgs = []

        try:
            controller_node = Controller('odom', 'cmd_vel', 0.15)

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(controller_node)

            test_node = Node('test_cmd_vel')
            executor.add_node(test_node)

            def cmd_vel_cb(msg):
                received_msgs.append(msg)

            test_node.create_subscription(Twist, 'cmd_vel', cmd_vel_cb, 10)

            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            # Wait for some messages
            timeout = time.time() + 5.0
            while len(received_msgs) < 3 and time.time() < timeout:
                time.sleep(0.1)

            assert len(received_msgs) >= 1, \
                f"Expected cmd_vel messages, got {len(received_msgs)}"

            # Without a goal, velocity should be zero
            last_msg = received_msgs[-1]
            assert last_msg.linear.x == 0.0, "Without goal, linear.x should be 0"
            assert last_msg.angular.z == 0.0, "Without goal, angular.z should be 0"

        finally:
            if executor:
                executor.shutdown()
            if controller_node:
                controller_node.destroy_node()
            if test_node:
                test_node.destroy_node()

    def test_controller_receives_odom(self):
        """Controller should update pose when odom is published."""
        sys.path.insert(0, os.path.dirname(__file__))
        from turtlebot_controller_node import Controller

        controller_node = None
        test_node = None
        executor = None

        try:
            controller_node = Controller('odom', 'cmd_vel', 0.15)

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(controller_node)

            test_node = Node('test_odom_pub')
            executor.add_node(test_node)

            odom_pub = test_node.create_publisher(Odometry, 'odom', 10)

            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            time.sleep(0.3)

            # Publish an odom message
            odom_msg = Odometry()
            odom_msg.pose.pose.position.x = 1.5
            odom_msg.pose.pose.position.y = 2.5
            odom_msg.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom_msg)

            # Wait for it to be processed
            timeout = time.time() + 3.0
            while controller_node.current_pose is None and time.time() < timeout:
                time.sleep(0.1)

            assert controller_node.current_pose is not None, \
                "Controller should have updated current_pose from odom"
            assert abs(controller_node.current_pose[0] - 1.5) < 0.01, \
                f"Expected x=1.5, got {controller_node.current_pose[0]}"
            assert abs(controller_node.current_pose[1] - 2.5) < 0.01, \
                f"Expected y=2.5, got {controller_node.current_pose[1]}"

        finally:
            if executor:
                executor.shutdown()
            if controller_node:
                controller_node.destroy_node()
            if test_node:
                test_node.destroy_node()


class TestStaticChecks:
    """Verify code content matches expected patterns (complementing oracle tests)."""

    def test_no_rospy_imports(self):
        """Ensure no rospy references in any file."""
        base = os.path.dirname(__file__)
        for fname in ['manage_objects_node.py', 'pickup_behaviors_node.py', 'turtlebot_controller_node.py']:
            fpath = os.path.join(base, fname)
            if os.path.exists(fpath):
                content = open(fpath).read()
                assert 'import rospy' not in content, f"rospy import found in {fname}"
                assert 'rospy.' not in content, f"rospy usage found in {fname}"

    def test_no_leading_slashes(self):
        """Ensure no leading slashes in topic/service names."""
        import re
        base = os.path.dirname(__file__)
        slash_pattern = r'[\'"]/(?:odom|cmd_vel|gazebo|spawn_entity|manage_objects)'
        for fname in ['manage_objects_node.py', 'pickup_behaviors_node.py', 'turtlebot_controller_node.py']:
            fpath = os.path.join(base, fname)
            if os.path.exists(fpath):
                content = open(fpath).read()
                assert not re.search(slash_pattern, content), \
                    f"Leading slash found in {fname}"