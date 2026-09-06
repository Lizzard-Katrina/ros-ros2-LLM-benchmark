"""
Runtime test for task_002_ur5_pick_and_place.

Tests the structural correctness and ROS2 compatibility of the migrated code
by verifying:
1. ArmController node can be instantiated and publishes trajectory messages
2. MotionPlanner node can be instantiated with correct service clients
3. Trajectory interpolation uses slerp and loop-based stepping
4. No nested deadlocks in class methods
"""

import pytest
import time
import sys
import os
import threading
import rclpy
from rclpy.node import Node
import trajectory_msgs.msg

# Add the package directory to sys.path so we can import the modules
pkg_dir = os.path.dirname(os.path.abspath(__file__))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class TrajectoryListener(Node):
    """Helper node that subscribes to trajectory commands."""
    def __init__(self):
        super().__init__('trajectory_listener')
        self.received_msgs = []
        self.sub = self.create_subscription(
            trajectory_msgs.msg.JointTrajectory,
            '/trajectory_controller/command',
            self._callback,
            50)

    def _callback(self, msg):
        self.received_msgs.append(msg)


def test_arm_controller_instantiation_and_publish():
    """
    Test that ArmController can be created as a ROS2 node and that
    move_to publishes multiple trajectory messages (interpolation).
    """
    from controller import ArmController
    from quaternion_utils import Quaternion

    listener = TrajectoryListener()
    controller = None

    try:
        # Create controller - it will use default pose since no real controller state topic
        controller = ArmController()

        # Verify it's a proper ROS2 node
        assert isinstance(controller, Node), "ArmController must be a ROS2 Node"

        # Verify joint names
        assert len(controller.joint_names) == 6
        assert "shoulder_pan_joint" in controller.joint_names

        # Verify publisher exists
        assert controller.joints_pub is not None

        # Verify gripper_pose is set
        assert hasattr(controller, 'gripper_pose')
        pos, quat = controller.gripper_pose
        assert len(pos) == 3

        # Now test move_to publishes messages
        target_quat = Quaternion(axis=(0, 1, 0), angle=3.14159)

        # Start spinning listener in background
        spin_done = threading.Event()

        def spin_listener():
            while not spin_done.is_set():
                rclpy.spin_once(listener, timeout_sec=0.05)

        t = threading.Thread(target=spin_listener, daemon=True)
        t.start()

        # Execute a move_to - this should publish multiple trajectory points
        # Use the current position to avoid kinematics issues
        cx, cy, cz = controller.gripper_pose[0]
        controller.move_to(cx, cy, cz, target_quat=target_quat, blocking=False)

        # Give time for messages to be received
        time.sleep(0.5)
        spin_done.set()
        t.join(timeout=2.0)

        # Verify multiple messages were published (interpolation, not single point)
        assert len(listener.received_msgs) > 1, \
            f"Expected multiple trajectory messages from interpolation, got {len(listener.received_msgs)}"

        # Verify message structure
        msg = listener.received_msgs[0]
        assert len(msg.joint_names) == 6
        assert len(msg.points) > 0
        assert len(msg.points[0].positions) == 6

        # Verify gripper_pose was updated
        new_pos, new_quat = controller.gripper_pose
        assert len(new_pos) == 3

    finally:
        if controller is not None:
            controller.destroy_node()
        listener.destroy_node()


def test_motion_planner_service_clients():
    """
    Test that MotionPlanner has correctly named service clients.
    """
    from motion_planning import MotionPlanner

    planner = None
    try:
        planner = MotionPlanner()

        # Verify it's a proper ROS2 node
        assert isinstance(planner, Node), "MotionPlanner must be a ROS2 Node"

        # Verify service clients exist with correct names
        assert hasattr(planner, 'setstatic_srv'), "Missing self.setstatic_srv"
        assert hasattr(planner, 'attach_srv'), "Missing self.attach_srv"
        assert hasattr(planner, 'detach_srv'), "Missing self.detach_srv"

        # Verify action client attribute exists
        assert hasattr(planner, 'action_gripper'), "Missing action_gripper"

        # Verify critical methods exist
        assert callable(getattr(planner, 'straighten', None))
        assert callable(getattr(planner, 'open_gripper', None))
        assert callable(getattr(planner, 'close_gripper', None))
        assert callable(getattr(planner, 'set_model_fixed', None))

    finally:
        if planner is not None:
            planner.destroy_node()


def test_controller_source_has_slerp_and_loop():
    """
    Verify that the controller source code contains slerp interpolation
    and a loop for trajectory generation.
    """
    import re

    ctrl_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'controller.py')
    with open(ctrl_file, 'r') as f:
        content = f.read()

    # Check for slerp
    assert 'slerp' in content.lower(), "Missing Slerp interpolation in controller"

    # Check for loop with np.arange or range
    has_loop = re.search(r"for\s+.*?\s+in\s+(np\.arange|range)", content)
    assert has_loop, "Missing interpolation loop in move_to"

    # Check gripper_pose update
    assert re.search(r"self\.gripper_pose\s*=", content), \
        "Controller must update self.gripper_pose"


def test_motion_planning_source_has_required_elements():
    """
    Verify motion_planning.py has all required orchestration elements.
    """
    mp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motion_planning.py')
    with open(mp_file, 'r') as f:
        content = f.read()

    # Check critical steps
    for step in ["straighten", "move_to", "open_gripper", "set_model_fixed"]:
        assert step in content, f"Missing critical step '{step}'"

    # Check service client names
    assert "self.attach_srv" in content
    assert "self.detach_srv" in content
    assert "self.setstatic_srv" in content

    # Check for error handling with continue
    assert "continue" in content
    assert "ValueError" in content or "except" in content

    # Check INTERLOCKING_OFFSET
    assert "INTERLOCKING_OFFSET" in content

    # Check DEFAULT_QUAT and PyQuaternion
    assert "DEFAULT_QUAT" in content
    assert "PyQuaternion" in content

    # Check ArmController usage
    assert "ArmController(" in content


def test_no_nested_spin_in_methods():
    """
    Verify no spin_until_future_complete inside class methods (self methods).
    """
    for filename in ['controller.py', 'motion_planning.py']:
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        with open(filepath, 'r') as f:
            content = f.read()

        lines = content.splitlines()
        in_method = False
        for line_num, line in enumerate(lines, 1):
            clean_line = line.strip()
            if clean_line.startswith("def ") and "self" in clean_line:
                in_method = True
                continue
            if in_method and "rclpy.spin_until_future_complete" in clean_line:
                if not clean_line.startswith("#"):
                    pytest.fail(
                        f"Detected 'spin_until_future_complete' inside a method "
                        f"in {filename} at line {line_num}. This causes deadlocks."
                    )
            if in_method and len(line) > 0 and not line.startswith(" ") and not line.startswith("\t"):
                in_method = False