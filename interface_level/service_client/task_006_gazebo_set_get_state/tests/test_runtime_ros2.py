"""
Runtime test for task_006_gazebo_set_get_state.

We verify:
1. The C++ source file compiles (already handled by colcon build).
2. The node executable exists and can be launched.
3. We create a mock service server for /gazebo/set_model_state and verify
   the node actually calls it with the expected fields.
4. We also publish on /gazebo/model_states and /gazebo/link_states to
   exercise the subscription callbacks.
"""
import subprocess
import time
import os
import signal
import threading
import pytest

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg import Pose, Twist


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class MockServiceServer(Node):
    """A mock /gazebo/set_model_state service server that records requests."""

    def __init__(self):
        super().__init__("mock_set_model_state_server")
        self.received_requests = []
        self.srv = self.create_service(
            SetModelState, "/gazebo/set_model_state", self.handle_request
        )

    def handle_request(self, request, response):
        self.received_requests.append(request)
        response.success = True
        response.status_message = "OK"
        return response


class TopicPublisher(Node):
    """Publishes on /gazebo/model_states and /gazebo/link_states."""

    def __init__(self):
        super().__init__("mock_gazebo_topic_publisher")
        self.model_pub = self.create_publisher(ModelStates, "/gazebo/model_states", 10)
        self.link_pub = self.create_publisher(LinkStates, "/gazebo/link_states", 10)

    def publish_once(self):
        ms = ModelStates()
        ms.name = ["ground_plane", "ball"]
        p0 = Pose()
        p1 = Pose()
        p1.position.x = 1.0
        ms.pose = [p0, p1]
        ms.twist = [Twist(), Twist()]
        self.model_pub.publish(ms)

        ls = LinkStates()
        ls.name = ["ground_plane::link", "ball::body"]
        lp0 = Pose()
        lp1 = Pose()
        lp1.position.z = 2.0
        ls.pose = [lp0, lp1]
        ls.twist = [Twist(), Twist()]
        self.link_pub.publish(ls)


def test_node_calls_set_model_state_service():
    """Launch the compiled node and verify it calls the mock service correctly."""
    mock_server = None
    topic_pub = None
    proc = None
    spin_thread = None
    stop_spinning = False

    try:
        mock_server = MockServiceServer()
        topic_pub = TopicPublisher()

        # Spin mock server in a background thread so it can respond to service calls
        def spin_nodes():
            while not stop_spinning and rclpy.ok():
                rclpy.spin_once(mock_server, timeout_sec=0.05)
                topic_pub.publish_once()
                rclpy.spin_once(topic_pub, timeout_sec=0.01)

        spin_thread = threading.Thread(target=spin_nodes, daemon=True)
        spin_thread.start()

        # Give the mock server a moment to be discoverable
        time.sleep(0.5)

        # Launch the node under test
        proc = subprocess.Popen(
            ["ros2", "run", "task_006_gazebo_set_get_state", "gazebo_model_states"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for the process to finish (it should exit after the service call)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)

        # Give a moment for the spin thread to pick up any last requests
        time.sleep(0.5)

        stderr_text = proc.stderr.read().decode()

        # Check that the service was actually called
        assert len(mock_server.received_requests) > 0, (
            f"Node did not call /gazebo/set_model_state service within timeout. "
            f"stderr: {stderr_text}"
        )

        req = mock_server.received_requests[0]
        assert req.model_state.model_name == "ball", (
            f"Expected model_name='ball', got '{req.model_state.model_name}'"
        )
        assert req.model_state.reference_frame == "world", (
            f"Expected reference_frame='world', got '{req.model_state.reference_frame}'"
        )
        # Check pose z == 1.0 (as set in the source)
        assert abs(req.model_state.pose.position.z - 1.0) < 1e-6, (
            f"Expected pose.position.z=1.0, got {req.model_state.pose.position.z}"
        )
        # Check twist is zero
        assert abs(req.model_state.twist.linear.x) < 1e-6
        assert abs(req.model_state.twist.angular.z) < 1e-6

        # Verify the node reported success (check stderr for the success message)
        assert "Setting position of ball model was successful" in stderr_text, (
            f"Expected success message in stderr. Got: {stderr_text}"
        )

    finally:
        stop_spinning = True
        if spin_thread is not None:
            spin_thread.join(timeout=3)
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if mock_server is not None:
            mock_server.destroy_node()
        if topic_pub is not None:
            topic_pub.destroy_node()