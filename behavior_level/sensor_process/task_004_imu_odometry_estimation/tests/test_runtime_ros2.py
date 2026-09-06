"""
Runtime test for the UKF projectSigmaPoint implementation.
Launches the ukf_test_node and verifies the predicted state output.
"""
import subprocess
import time
import math
import os
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StateListener(Node):
    def __init__(self):
        super().__init__('state_listener')
        self.received_msg = None
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'ukf_test_output',
            self.callback,
            10
        )

    def callback(self, msg):
        self.received_msg = msg


def test_ukf_predict_3d():
    """
    Test that the UKF prediction correctly projects state through 3D kinematics.
    We set:
      Vx=1.0, Yaw=pi/4, Pitch=pi/6, Ax=0.5, dt=0.1
    Expected X displacement (approx):
      X += cos(yaw)*cos(pitch)*Vx*dt + 0.5*cos(yaw)*cos(pitch)*Ax*dt^2
      X += cos(pi/4)*cos(pi/6)*1.0*0.1 + 0.5*cos(pi/4)*cos(pi/6)*0.5*0.01
      X += 0.7071*0.8660*0.1 + 0.5*0.7071*0.8660*0.005
      X += 0.06124 + 0.001531 ~ 0.06277
    The UKF uses sigma points so the result will be close but not exact.
    We check that X > 0 (moved forward) and is in a reasonable range.
    """
    rclpy.init()
    proc = None
    listener = None
    try:
        # Launch the test node
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_004_imu_odometry_estimation', 'ukf_test_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        listener = StateListener()

        # Wait for the message with timeout
        timeout = 15.0
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(listener, timeout_sec=0.1)
            if listener.received_msg is not None:
                break

        assert listener.received_msg is not None, "Did not receive UKF test output within timeout"

        data = listener.received_msg.data
        assert len(data) == 15, f"Expected 15 state elements, got {len(data)}"

        # StateMemberX = 0, StateMemberY = 1, StateMemberZ = 2
        # StateMemberRoll = 3, StateMemberPitch = 4, StateMemberYaw = 5
        x_val = data[0]
        y_val = data[1]
        z_val = data[2]
        roll_val = data[3]
        pitch_val = data[4]
        yaw_val = data[5]

        # With Vx=1.0, yaw=pi/4, pitch=pi/6, dt=0.1:
        # X should be positive and roughly 0.06
        assert x_val > 0.01, f"X position should be positive and significant, got {x_val}"
        assert x_val < 0.2, f"X position should be reasonable, got {x_val}"

        # Y should also be positive (yaw=pi/4 means sin(yaw)>0)
        assert y_val > 0.01, f"Y position should be positive, got {y_val}"
        assert y_val < 0.2, f"Y position should be reasonable, got {y_val}"

        # Z should be negative (pitch=pi/6 means -sin(pitch)*Vx contribution)
        assert z_val < -0.001, f"Z position should be negative due to pitch, got {z_val}"
        assert z_val > -0.2, f"Z position should be reasonable, got {z_val}"

        # Yaw should have changed from pi/4 due to Vyaw=0.1
        assert abs(yaw_val - (math.pi / 4.0)) < 0.15, \
            f"Yaw should be close to pi/4, got {yaw_val}"

        # Pitch should have changed slightly from pi/6 due to Vpitch=0.05
        assert abs(pitch_val - (math.pi / 6.0)) < 0.15, \
            f"Pitch should be close to pi/6, got {pitch_val}"

    finally:
        if listener is not None:
            listener.destroy_node()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=5)
        rclpy.shutdown()


def test_ukf_source_file_exists():
    """Verify the ukf.cpp source file is present in the installed share directory."""
    # Check in the source tree (where tests run from)
    ukf_path = os.path.join(os.path.dirname(__file__), 'ukf.cpp')
    if not os.path.exists(ukf_path):
        # Try installed share path
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('task_004_imu_odometry_estimation')
        ukf_path = os.path.join(share_dir, 'ukf.cpp')
    assert os.path.exists(ukf_path), f"ukf.cpp not found at {ukf_path}"


def test_source_uses_state_member_enums():
    """Verify the source uses StateMember enums, not hard-coded indices."""
    import re
    ukf_path = os.path.join(os.path.dirname(__file__), 'ukf.cpp')
    if not os.path.exists(ukf_path):
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('task_004_imu_odometry_estimation')
        ukf_path = os.path.join(share_dir, 'ukf.cpp')

    with open(ukf_path, 'r') as f:
        content = f.read()

    # Check for key StateMember usage
    assert 'StateMemberVx' in content, "Must use StateMemberVx enum"
    assert 'StateMemberAx' in content, "Must use StateMemberAx enum"
    assert 'StateMemberVpitch' in content, "Must use StateMemberVpitch enum"
    assert 'applyOnTheLeft' in content, "Must use applyOnTheLeft for efficiency"

    # Check no hard-coded indices in transfer_function_
    clean = re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.DOTALL)
    hardcoded = re.findall(r'transfer_function_\(\s*\d+\s*,\s*\d+\s*\)', clean)
    assert len(hardcoded) == 0, f"Found hard-coded indices: {hardcoded}"