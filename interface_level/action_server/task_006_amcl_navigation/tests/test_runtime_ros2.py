"""
Runtime test for the ROS2 AMCL navigation action server node.
Tests both static code analysis (oracle) and runtime behavior.
"""
import os
import re
import time
import signal
import subprocess
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time as TimeMsg


# -------------------------------------------------------------------
# Static oracle tests (must keep passing)
# -------------------------------------------------------------------

def _load_code():
    """Load the translated ROS2 C++ code."""
    # Try several possible locations
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', 'amcl_node.cpp'),
        os.path.join(os.path.dirname(__file__), 'amcl_node.cpp'),
    ]
    # Also search recursively from test dir parent
    search_root = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        search_root = os.path.dirname(search_root)
        for root, dirs, files in os.walk(search_root):
            if 'amcl_node.cpp' in files:
                candidates.append(os.path.join(root, 'amcl_node.cpp'))

    for fp in candidates:
        if os.path.exists(fp):
            with open(fp, 'r') as f:
                return f.read()

    pytest.fail("amcl_node.cpp not found")


@pytest.fixture
def code():
    return _load_code()


def test_action_server_creation(code):
    """Check if a ROS2 Action Server is defined."""
    pattern = r"rclcpp_action::create_server<\s*UpdatePose\s*>"
    assert re.search(pattern, code), \
        "Missing ROS2 Action Server creation for UpdatePose action."


def test_handle_goal_defined(code):
    """Check if handle_goal callback exists."""
    pattern = r"handle_goal\s*\("
    assert re.search(pattern, code), \
        "handle_goal callback function is not implemented."


def test_handle_cancel_defined(code):
    """Check if handle_cancel callback exists."""
    pattern = r"handle_cancel\s*\("
    assert re.search(pattern, code), \
        "handle_cancel callback function is not implemented."


def test_handle_accepted_defined(code):
    """Check if handle_accepted callback exists."""
    pattern = r"handle_accepted\s*\("
    assert re.search(pattern, code), \
        "handle_accepted callback function is not implemented."


def test_todo_comment_present(code):
    """Check if TODO for particle filter update is present."""
    pattern = r"//\s*TODO: Implement particle filter update"
    assert re.search(pattern, code), \
        "Missing TODO comment for particle filter update."


def test_thread_usage_for_async(code):
    """Check if async handling (std::thread) is used in handle_accepted."""
    pattern = r"std::thread\s*\("
    assert re.search(pattern, code), \
        "Async thread handling for action execution missing."


def test_feedback_or_result_mentioned(code):
    """Check if action result or feedback is defined."""
    pattern = r"(Feedback|Result|goal_handle->succeed)"
    assert re.search(pattern, code), \
        "Action feedback/result handling missing in handle_accepted."


def test_laser_data_handling_mentioned(code):
    """Check if code mentions laser data ranges/bearings."""
    pattern = r"(ranges|bearing)"
    assert re.search(pattern, code), \
        "Laser sensor ranges or bearings handling not mentioned."


def test_resample_called(code):
    """Check if particle filter resampling is indicated."""
    pattern = r"(resample|pf_update_resample)"
    assert re.search(pattern, code), \
        "Particle filter resampling step not found."


def test_pose_publishing_mentioned(code):
    """Check if publishing pose is mentioned."""
    pattern = r"(publish|pose_pub_|\bPoseWithCovarianceStamped\b)"
    assert re.search(pattern, code), \
        "Publishing of pose not implemented or mentioned."


# -------------------------------------------------------------------
# Runtime test: verify the node can be launched and action server exists
# -------------------------------------------------------------------

class TestAmclRuntime:
    """Runtime tests that launch the node and interact with it."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        """Setup and teardown for runtime tests."""
        self.proc = None
        self.node = None
        rclpy.init()
        yield
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _start_node(self):
        """Start the amcl_node as a subprocess."""
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'task_006_amcl_navigation', 'amcl_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        # Give the node time to start
        time.sleep(3.0)
        assert self.proc.poll() is None, \
            f"Node exited prematurely with code {self.proc.returncode}"

    def test_node_launches_and_action_server_available(self):
        """Test that the node launches and the action server is discoverable."""
        self._start_node()

        self.node = Node('test_amcl_client')

        # Import the generated action type
        from task_006_amcl_navigation.action import UpdatePose
        action_client = ActionClient(self.node, UpdatePose, 'update_pose')

        server_found = action_client.wait_for_server(timeout_sec=8.0)
        assert server_found, "Action server 'update_pose' not found within timeout"

        action_client.destroy()

    def test_send_goal_and_get_result(self):
        """Test sending a goal to the action server and getting a result."""
        self._start_node()

        self.node = Node('test_amcl_goal_client')

        from task_006_amcl_navigation.action import UpdatePose
        action_client = ActionClient(self.node, UpdatePose, 'update_pose')
        assert action_client.wait_for_server(timeout_sec=8.0), \
            "Action server not available"

        # Create and send a goal
        goal_msg = UpdatePose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 2.0
        goal_msg.pose.pose.orientation.w = 1.0

        future = action_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        assert future.done(), "Goal send did not complete in time"

        goal_handle = future.result()
        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted"

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        assert result_future.done(), "Result not received in time"

        result = result_future.result()
        assert result is not None, "Result is None"
        # The action should have succeeded (status 4 = STATUS_SUCCEEDED)
        assert result.status == 4, \
            f"Expected status SUCCEEDED (4), got {result.status}"

        # Check result pose has the expected frame_id
        assert result.result.result_pose.header.frame_id == 'map', \
            f"Expected frame_id 'map', got '{result.result.result_pose.header.frame_id}'"

        # ---- Numeric checks on the pose computed by the (blanked) filter cycle ----
        # The groundtruth pose-update cycle emits the node's current estimate,
        # which is seeded from init_pose_ = (0, 0, 0) with init_cov_ =
        # (0.5^2, 0.5^2, (pi/12)^2). A stubbed-out execute()/filter cycle cannot
        # produce these exact numbers, so assert them explicitly.
        import math
        rp = result.result.result_pose.pose
        assert abs(rp.position.x - 0.0) < 1e-6, f"result pose x: {rp.position.x}"
        assert abs(rp.position.y - 0.0) < 1e-6, f"result pose y: {rp.position.y}"
        assert abs(rp.position.z - 0.0) < 1e-6, f"result pose z: {rp.position.z}"
        # yaw 0 -> identity quaternion
        assert abs(rp.orientation.z - 0.0) < 1e-6, f"result quat z: {rp.orientation.z}"
        assert abs(abs(rp.orientation.w) - 1.0) < 1e-6, f"result quat w: {rp.orientation.w}"

        action_client.destroy()

    def test_pose_publisher_active(self):
        """Test that the amcl_pose topic is being published after action."""
        self._start_node()

        self.node = Node('test_pose_subscriber')
        received_msgs = []

        def pose_cb(msg):
            received_msgs.append(msg)

        sub = self.node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', pose_cb, 10)

        # Also send a goal to trigger pose publishing
        from task_006_amcl_navigation.action import UpdatePose
        action_client = ActionClient(self.node, UpdatePose, 'update_pose')
        if action_client.wait_for_server(timeout_sec=5.0):
            goal_msg = UpdatePose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = 0.0
            goal_msg.pose.pose.position.y = 0.0
            goal_msg.pose.pose.orientation.w = 1.0
            send_future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

            if send_future.done() and send_future.result() is not None:
                gh = send_future.result()
                if gh.accepted:
                    res_future = gh.get_result_async()
                    rclpy.spin_until_future_complete(self.node, res_future, timeout_sec=10.0)

        # Spin for a bit to receive messages
        start = time.time()
        while time.time() - start < 5.0 and len(received_msgs) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        # The pose should have been published after the action executed
        assert len(received_msgs) > 0, \
            "No PoseWithCovarianceStamped messages received on amcl_pose topic"

        msg = received_msgs[0]
        assert msg.header.frame_id == 'map', \
            f"Expected frame_id 'map', got '{msg.header.frame_id}'"

        # Numeric checks: the published estimate is seeded from init_pose_ = 0
        # with init_cov_ = (0.25, 0.25, (pi/12)^2) on the yaw entry.
        import math
        assert abs(msg.pose.pose.position.x) < 1e-6, msg.pose.pose.position.x
        assert abs(msg.pose.pose.position.y) < 1e-6, msg.pose.pose.position.y
        assert abs(abs(msg.pose.pose.orientation.w) - 1.0) < 1e-6, msg.pose.pose.orientation.w
        assert abs(msg.pose.covariance[0] - 0.25) < 1e-6, msg.pose.covariance[0]
        assert abs(msg.pose.covariance[7] - 0.25) < 1e-6, msg.pose.covariance[7]
        assert abs(msg.pose.covariance[35] - (math.pi / 12.0) ** 2) < 1e-6, msg.pose.covariance[35]

        self.node.destroy_subscription(sub)
        action_client.destroy()

    def test_laser_scan_triggers_pose_update(self):
        """Publish a LaserScan and require the (blanked) laserReceived() filter
        cycle to emit an estimated pose on amcl_pose.

        No action goal is sent here, so the only code path that can publish on
        amcl_pose is laserReceived(). A stubbed-out laserReceived() produces no
        message and this test fails.
        """
        self._start_node()

        self.node = Node('test_laser_driver')
        received = []
        sub = self.node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', lambda m: received.append(m), 10)

        scan_pub = self.node.create_publisher(LaserScan, 'scan', 10)

        # Let discovery settle.
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        scan = LaserScan()
        scan.header.frame_id = 'base_laser'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 3.14 / 180.0
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [5.0] * 181

        # Publish several scans; first one initialises the filter and forces a publish.
        for _ in range(10):
            scan.header.stamp = self.node.get_clock().now().to_msg()
            scan_pub.publish(scan)
            end = time.time() + 0.5
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if received:
                break

        assert len(received) > 0, \
            "laserReceived() produced no amcl_pose message for an incoming LaserScan"
        msg = received[0]
        assert msg.header.frame_id == 'map', \
            f"Expected frame_id 'map', got '{msg.header.frame_id}'"
        assert abs(msg.pose.pose.position.x) < 1e-6, msg.pose.pose.position.x
        assert abs(msg.pose.pose.position.y) < 1e-6, msg.pose.pose.position.y
        assert abs(abs(msg.pose.pose.orientation.w) - 1.0) < 1e-6, msg.pose.pose.orientation.w

        self.node.destroy_subscription(sub)