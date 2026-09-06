"""
Runtime test for task_012_px4_flight_params.
Tests that the OffboardControl node correctly declares and retrieves parameters,
and that the node can be instantiated and its parameters queried.
"""
import pytest
import time
import json
import rclpy
from rclpy.node import Node


def test_node_parameters_default():
    """Test that the node declares parameters with correct default values."""
    rclpy.init()
    try:
        from task_012_px4_flight_params.offboard_control import OffboardControl

        node = OffboardControl()
        try:
            # Verify parameters are declared and have correct default values
            takeoff_height = node.get_parameter('takeoff_height').value
            target_yaw = node.get_parameter('target_yaw').value

            assert takeoff_height == -5.0, f"Expected takeoff_height=-5.0, got {takeoff_height}"
            assert abs(target_yaw - 1.57079) < 1e-4, f"Expected target_yaw~1.57079, got {target_yaw}"

            # Verify class attributes are set from parameters
            assert node.takeoff_height == -5.0, f"Expected self.takeoff_height=-5.0, got {node.takeoff_height}"
            assert abs(node.target_yaw - 1.57079) < 1e-4, f"Expected self.target_yaw~1.57079, got {node.target_yaw}"

            # Verify the node has the expected publishers
            assert node.offboard_control_mode_publisher is not None
            assert node.trajectory_setpoint_publisher is not None
            assert node.vehicle_command_publisher is not None

        finally:
            node.timer.cancel()
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_node_parameters_custom():
    """Test that the node accepts custom parameter values."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'takeoff_height:=-10.0',
        '-p', 'target_yaw:=3.14159',
    ])
    try:
        from task_012_px4_flight_params.offboard_control import OffboardControl

        node = OffboardControl()
        try:
            takeoff_height = node.get_parameter('takeoff_height').value
            target_yaw = node.get_parameter('target_yaw').value

            assert takeoff_height == -10.0, f"Expected takeoff_height=-10.0, got {takeoff_height}"
            assert abs(target_yaw - 3.14159) < 1e-4, f"Expected target_yaw~3.14159, got {target_yaw}"

            # Verify class attributes reflect custom values
            assert node.takeoff_height == -10.0, f"Expected self.takeoff_height=-10.0, got {node.takeoff_height}"
            assert abs(node.target_yaw - 3.14159) < 1e-4, f"Expected self.target_yaw~3.14159, got {node.target_yaw}"

        finally:
            node.timer.cancel()
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_trajectory_setpoint_publication():
    """Test that TrajectorySetpoint messages are published with correct field values."""
    rclpy.init()
    try:
        from task_012_px4_flight_params.offboard_control import (
            OffboardControl, _decode_trajectory_setpoint, HAS_PX4_MSGS
        )
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        if HAS_PX4_MSGS:
            from px4_msgs.msg import TrajectorySetpoint as TrajectorySetpointMsg
        else:
            from std_msgs.msg import String as TrajectorySetpointMsg

        node = OffboardControl()
        # Cancel the timer so it doesn't interfere
        node.timer.cancel()

        received_msgs = []

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        test_node = rclpy.create_node('test_subscriber_node')

        def _cb(raw_msg):
            decoded = _decode_trajectory_setpoint(raw_msg)
            received_msgs.append(decoded)

        sub = test_node.create_subscription(
            TrajectorySetpointMsg,
            '/fmu/in/trajectory_setpoint',
            _cb,
            qos_profile
        )

        try:
            # Publish a position setpoint
            node.publish_position_setpoint(1.0, 2.0, node.takeoff_height)

            # Spin both nodes to allow message delivery
            timeout = time.time() + 5.0
            while time.time() < timeout and len(received_msgs) == 0:
                rclpy.spin_once(node, timeout_sec=0.05)
                rclpy.spin_once(test_node, timeout_sec=0.05)

            assert len(received_msgs) > 0, "No TrajectorySetpoint messages received"

            msg = received_msgs[0]
            # Check position array
            assert abs(msg.position[0] - 1.0) < 1e-4, f"position[0] expected 1.0, got {msg.position[0]}"
            assert abs(msg.position[1] - 2.0) < 1e-4, f"position[1] expected 2.0, got {msg.position[1]}"
            assert abs(msg.position[2] - (-5.0)) < 1e-4, f"position[2] expected -5.0, got {msg.position[2]}"

            # Check yaw from parameter
            assert abs(msg.yaw - 1.57079) < 1e-4, f"yaw expected ~1.57079, got {msg.yaw}"

            # Check timestamp is in microseconds (should be > 0 and reasonable)
            assert msg.timestamp > 0, f"timestamp should be > 0, got {msg.timestamp}"

        finally:
            test_node.destroy_node()
            node.destroy_node()
    finally:
        rclpy.shutdown()