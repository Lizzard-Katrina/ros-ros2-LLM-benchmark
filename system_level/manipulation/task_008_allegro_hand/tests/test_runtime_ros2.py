import os
import sys
import time
import signal
import subprocess
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_allegro_node_publishes_joint_states(ros_context):
    """
    Launch the allegro_node executable and verify it publishes
    JointState messages on the expected topic with correct structure.
    """
    proc = None
    received_msgs = []

    class Listener(Node):
        def __init__(self):
            super().__init__('test_listener')
            self.sub = self.create_subscription(
                JointState,
                'allegroHand/joint_states',
                self.cb,
                10
            )

        def cb(self, msg):
            received_msgs.append(msg)

    listener = None
    try:
        # Create listener FIRST so subscription is ready before node starts publishing
        listener = Listener()

        # Launch the node as subprocess
        env = os.environ.copy()
        proc = subprocess.Popen(
            ['ros2', 'run', 'task_008_allegro_hand', 'allegro_node'],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Spin for up to 8 seconds waiting for messages
        timeout = time.time() + 8.0
        while time.time() < timeout and len(received_msgs) < 5:
            rclpy.spin_once(listener, timeout_sec=0.1)

        # Verify we received messages
        assert len(received_msgs) > 0, "No JointState messages received from allegro_node"

        msg = received_msgs[-1]  # Use the latest message

        # Check the message has 16 joints
        assert len(msg.name) == 16, f"Expected 16 joint names, got {len(msg.name)}"
        assert len(msg.position) == 16, f"Expected 16 positions, got {len(msg.position)}"
        assert len(msg.velocity) == 16, f"Expected 16 velocities, got {len(msg.velocity)}"
        assert len(msg.effort) == 16, f"Expected 16 efforts, got {len(msg.effort)}"

        # Check joint names match expected
        expected_names = [
            "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
            "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
            "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
            "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        ]
        for i, name in enumerate(expected_names):
            assert msg.name[i] == name, f"Joint name mismatch at index {i}: {msg.name[i]} != {name}"

        # Check that positions are the injected test values (0.1 * (i+1))
        for i in range(16):
            expected_val = 0.1 * (i + 1)
            assert abs(msg.position[i] - expected_val) < 1e-6, \
                f"Position[{i}] = {msg.position[i]}, expected {expected_val}"

    finally:
        if listener:
            listener.destroy_node()
        if proc:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def test_source_files_exist():
    """Verify the source files exist and contain expected patterns (sanity check)."""
    import re
    from pathlib import Path

    pkg_dir = Path(__file__).resolve().parents[0]
    drv_cpp = pkg_dir / "src" / "AllegroHandDrv.cpp"
    node_cpp = pkg_dir / "src" / "allegro_node.cpp"

    assert drv_cpp.exists(), f"Driver source not found: {drv_cpp}"
    assert node_cpp.exists(), f"Node source not found: {node_cpp}"

    drv_content = drv_cpp.read_text()
    node_content = node_cpp.read_text()

    # Check driver patterns
    assert re.search(r"data\[.*?\]\s*\|\s*\(?\s*data\[.*?\]\s*<<\s*8\s*\)?", drv_content), \
        "Missing bit-shift assembly in driver"
    assert re.search(r"0\.088", drv_content), "Missing 0.088 scaling constant"
    assert re.search(r"M_PI\s*/\s*180\.0", drv_content), "Missing degree-to-radian conversion"
    assert re.search(r"_curr_position_get\s*\|=\s*\(\s*0x01\s*<<\s*findex\s*\)", drv_content), \
        "Missing bitmask update"

    # Check node patterns
    assert re.search(r"canDevice->getJointInfo", node_content), "Missing getJointInfo call"
    assert re.search(r"computeDesiredTorque\(", node_content), "Missing computeDesiredTorque call"
    assert re.search(r"canDevice->resetJointInfoReady\(", node_content), "Missing resetJointInfoReady call"
    assert re.search(r"previous_position\[.*?\]\s*=\s*current_position\[.*?\]", node_content), \
        "Missing position backup"
    assert re.search(r"\(\s*current_position\[.*?\]\s*-\s*previous_position\[.*?\]\s*\)\s*/\s*dt", node_content), \
        "Missing velocity calculation"