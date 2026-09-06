"""
Runtime test for the translated JointTrajectoryExecuter action server.
Launches the actual compiled node, sends an action goal, and verifies
that the trajectory is published on the 'command' topic.
"""
import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class CommandListener(Node):
    """Helper node that subscribes to 'command' topic to verify trajectory publication."""
    def __init__(self):
        super().__init__('test_command_listener')
        self.received_msgs = []
        self.sub = self.create_subscription(
            JointTrajectory,
            'command',
            self._cb,
            10
        )

    def _cb(self, msg):
        self.received_msgs.append(msg)


def test_action_server_publishes_trajectory():
    """
    Launch the real joint_trajectory_action_node executable, send it an action
    goal, and verify that the trajectory appears on the 'command' topic.
    """
    proc = None
    listener = None
    action_client_node = None

    try:
        # Launch the actual compiled node
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_003_pr2_gripper_action_server',
             'joint_trajectory_action_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the node time to start
        time.sleep(2.0)

        # Create a listener node for the 'command' topic
        listener = CommandListener()

        # Create an action client node
        action_client_node = Node('test_action_client')
        action_client = ActionClient(
            action_client_node,
            FollowJointTrajectory,
            'joint_trajectory_action'
        )

        # Wait for the action server to be available
        assert action_client.wait_for_server(timeout_sec=10.0), \
            "Action server not available within timeout"

        # Build a goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2']
        point = JointTrajectoryPoint()
        point.positions = [1.0, 2.0]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal_msg.trajectory.points.append(point)

        # Send the goal
        future = action_client.send_goal_async(goal_msg)

        # Spin until the goal is accepted or timeout
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(action_client_node, timeout_sec=0.1)
            rclpy.spin_once(listener, timeout_sec=0.1)
            if future.done():
                break

        assert future.done(), "send_goal_async did not complete in time"
        goal_handle = future.result()
        assert goal_handle is not None, "Goal handle is None"
        assert goal_handle.accepted, "Goal was not accepted"

        # Now spin the listener to pick up the published trajectory
        deadline = time.time() + 5.0
        while time.time() < deadline and len(listener.received_msgs) == 0:
            rclpy.spin_once(listener, timeout_sec=0.1)
            rclpy.spin_once(action_client_node, timeout_sec=0.05)

        # Verify that a trajectory was published on 'command'
        assert len(listener.received_msgs) > 0, \
            "No trajectory message received on 'command' topic"

        # Check the content of the published trajectory
        published = listener.received_msgs[-1]
        # The node has no joints parameter set, so it accepts any joint names
        assert len(published.points) > 0, "Published trajectory has no points"
        assert published.points[0].positions[0] == pytest.approx(1.0, abs=0.01)
        assert published.points[0].positions[1] == pytest.approx(2.0, abs=0.01)

    finally:
        if listener:
            listener.destroy_node()
        if action_client_node:
            action_client_node.destroy_node()
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def test_source_file_has_required_patterns():
    """
    Quick sanity check that the source file at the package root
    (used by the oracle test) contains the expected patterns.
    """
    from pathlib import Path
    src_file = Path(__file__).parent / "joint_trajectory_action.cpp"
    assert src_file.exists(), "joint_trajectory_action.cpp not found at package root"
    code = src_file.read_text()

    # Verify key ROS2 patterns
    assert "rclcpp_action" in code
    assert "rclcpp::Node" in code
    assert "JointTrajectoryAction" in code
    assert "goalCB" in code
    assert "controllerStateCB" in code
    assert "set_succeeded" in code
    assert "set_aborted" in code
    assert "set_accepted" in code
    assert "set_canceled" in code
    assert "publish" in code
    assert "TrajectoryControllerState" in code

    # Verify no ROS1 artifacts
    assert "ros/ros.h" not in code
    assert "ros::init" not in code
    assert "ros::NodeHandle" not in code
    assert "actionlib/server/action_server.h" not in code