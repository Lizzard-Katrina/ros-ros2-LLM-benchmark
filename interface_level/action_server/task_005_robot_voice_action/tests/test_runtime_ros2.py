"""
Runtime test for the offboard_control node from task_005_robot_voice_action.

This test:
1. Creates subscriptions FIRST, then launches the offboard_control executable
2. Subscribes to the published topics and verifies messages arrive with correct content
3. Checks that the node publishes OffboardControlMode, TrajectorySetpoint, and VehicleCommand
"""

import subprocess
import time
import threading
import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def test_offboard_control_publishes_messages():
    """
    Launch the offboard_control node and verify it publishes the expected
    messages on the correct topics with correct content.
    """
    rclpy.init()
    proc = None
    test_node = None

    try:
        # Create a test node and set up subscriptions BEFORE launching the target node
        test_node = rclpy.create_node("test_offboard_subscriber")

        # Use best_effort QoS to match the publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        offboard_msgs = []
        trajectory_msgs = []
        vehicle_cmd_msgs = []

        # Import the message types from our package
        from task_005_robot_voice_action.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
        )

        test_node.create_subscription(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            lambda msg: offboard_msgs.append(msg),
            qos,
        )

        test_node.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            lambda msg: trajectory_msgs.append(msg),
            qos,
        )

        test_node.create_subscription(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            lambda msg: vehicle_cmd_msgs.append(msg),
            qos,
        )

        # Start spinning in a background thread so we catch messages immediately
        spin_running = True

        def spin_thread():
            while spin_running:
                rclpy.spin_once(test_node, timeout_sec=0.05)

        spinner = threading.Thread(target=spin_thread, daemon=True)
        spinner.start()

        # Give the subscriptions a moment to be fully established
        time.sleep(0.5)

        # NOW launch the offboard_control node
        proc = subprocess.Popen(
            ["ros2", "run", "task_005_robot_voice_action", "offboard_control"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for messages to arrive. The VehicleCommand messages are published
        # at counter==10, which is ~1 second after the node starts (10 Hz timer).
        # We need to wait long enough for that to happen.
        timeout = time.time() + 10.0
        while time.time() < timeout:
            time.sleep(0.2)
            # Check if we have enough messages
            if (
                len(offboard_msgs) >= 3
                and len(trajectory_msgs) >= 3
                and len(vehicle_cmd_msgs) >= 1
            ):
                break

        # Stop the spinner
        spin_running = False
        spinner.join(timeout=2.0)

        # Verify OffboardControlMode messages
        assert len(offboard_msgs) >= 1, (
            f"Expected OffboardControlMode messages, got {len(offboard_msgs)}"
        )
        ocm = offboard_msgs[0]
        assert ocm.position is True, "OffboardControlMode.position should be True"
        assert ocm.velocity is False, "OffboardControlMode.velocity should be False"
        assert ocm.timestamp > 0, "OffboardControlMode.timestamp should be > 0"

        # Verify TrajectorySetpoint messages
        assert len(trajectory_msgs) >= 1, (
            f"Expected TrajectorySetpoint messages, got {len(trajectory_msgs)}"
        )
        ts = trajectory_msgs[0]
        assert abs(ts.position[0] - 0.0) < 0.01, "TrajectorySetpoint x should be 0.0"
        assert abs(ts.position[1] - 0.0) < 0.01, "TrajectorySetpoint y should be 0.0"
        assert (
            abs(ts.position[2] - (-5.0)) < 0.01
        ), "TrajectorySetpoint z should be -5.0"
        assert abs(ts.yaw - (-3.14159)) < 0.01, "TrajectorySetpoint yaw should be ~-pi"
        assert ts.timestamp > 0, "TrajectorySetpoint.timestamp should be > 0"

        # Verify VehicleCommand messages (arm + set_mode)
        assert len(vehicle_cmd_msgs) >= 1, (
            f"Expected VehicleCommand messages, got {len(vehicle_cmd_msgs)}"
        )
        # Check that at least one command has from_external = True
        has_external = any(msg.from_external for msg in vehicle_cmd_msgs)
        assert has_external, "VehicleCommand.from_external should be True"

        # Check that we got an arm command (command == 400, param1 == 1.0)
        arm_cmds = [
            msg
            for msg in vehicle_cmd_msgs
            if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            and msg.param1 == 1.0
        ]
        assert len(arm_cmds) >= 1, (
            f"Expected at least one ARM command, got commands: "
            f"{[(m.command, m.param1) for m in vehicle_cmd_msgs]}"
        )

        # Check that we got a set_mode command (command == 176)
        mode_cmds = [
            msg
            for msg in vehicle_cmd_msgs
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        ]
        assert len(mode_cmds) >= 1, (
            f"Expected at least one VEHICLE_CMD_DO_SET_MODE command, got commands: "
            f"{[(m.command, m.param1) for m in vehicle_cmd_msgs]}"
        )

        # Verify target_system is set
        for cmd in vehicle_cmd_msgs:
            assert cmd.target_system == 1, "VehicleCommand.target_system should be 1"
            assert (
                cmd.target_component == 1
            ), "VehicleCommand.target_component should be 1"

    finally:
        if test_node is not None:
            test_node.destroy_node()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        rclpy.try_shutdown()


def test_source_file_exists_and_is_valid():
    """
    Verify the source file exists at the expected location and contains
    key ROS2 patterns (complementary to the static oracle).
    """
    from pathlib import Path

    # Check in the package source directory
    src_file = Path(__file__).parent / "offboard_control.cpp"
    if not src_file.exists():
        src_file = Path(__file__).parent / "src" / "offboard_control.cpp"

    assert src_file.exists(), f"offboard_control.cpp not found"

    code = src_file.read_text()

    # Basic sanity checks that complement the static oracle
    assert "rclcpp::Node" in code, "Must inherit from rclcpp::Node"
    assert (
        "create_wall_timer" in code or "create_timer" in code
    ), "Must use ROS2 timer"
    assert "get_clock" in code, "Must use ROS2 clock"
    assert "OffboardControlMode" in code
    assert "TrajectorySetpoint" in code
    assert "VehicleCommand" in code