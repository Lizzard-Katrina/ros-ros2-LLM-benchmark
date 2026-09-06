#!/usr/bin/env python3
"""
Runtime test for task_007_autonomous_navigation.

Tests the Turtlebot3PatrolServer node by:
1. Launching the node
2. Publishing fake odometry data
3. Verifying cmd_vel messages are published with expected control behavior
4. Running the static oracle tests against the source code
"""

import math
import time
import threading
import pytest
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelListener(Node):
    """Helper node that subscribes to cmd_vel and publishes fake odom."""

    def __init__(self):
        super().__init__('test_listener_node')
        qos = QoSProfile(depth=10)

        self.received_twists = []
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)

        # Simulated position state
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_yaw = 0.0
        self.last_twist = Twist()
        self.last_time = time.time()

        # Timer to publish odom at 20 Hz
        self.odom_timer = self.create_timer(0.05, self.publish_odom)

    def cmd_vel_callback(self, msg):
        self.received_twists.append(msg)
        self.last_twist = msg

    def publish_odom(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Simple kinematic simulation
        v = self.last_twist.linear.x
        w = self.last_twist.angular.z

        self.sim_yaw += w * dt
        self.sim_x += v * math.cos(self.sim_yaw) * dt
        self.sim_y += v * math.sin(self.sim_yaw) * dt

        odom = Odometry()
        odom.pose.pose.position.x = self.sim_x
        odom.pose.pose.position.y = self.sim_y

        # Convert yaw to quaternion (only z and w components needed for 2D)
        odom.pose.pose.orientation.z = math.sin(self.sim_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.sim_yaw / 2.0)

        self.odom_pub.publish(odom)

    def clear(self):
        self.received_twists.clear()


class TestPatrolServerRuntime:
    """Runtime tests that verify the patrol server node behavior."""

    def test_node_starts_and_publishes_cmd_vel(self):
        """Test that the node starts and we can interact with it via topics."""
        rclpy.init()
        executor = None
        try:
            from task_007_autonomous_navigation.turtle_patrol_server import Turtlebot3PatrolServer

            server = Turtlebot3PatrolServer()
            listener = CmdVelListener()

            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor()
            executor.add_node(listener)

            # Spin listener in background so odom gets published
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            # Give time for odom to start flowing
            time.sleep(0.5)

            # Test go_front: drive 0.1 meters
            server.linear_x = 0.5
            listener.clear()

            go_thread = threading.Thread(
                target=server.go_front, args=(0.0, 0.1), daemon=True
            )
            go_thread.start()
            go_thread.join(timeout=10.0)

            # Small delay to ensure the final stop message is received by listener
            time.sleep(0.2)

            # Verify cmd_vel messages were published
            assert len(listener.received_twists) > 0, \
                "No cmd_vel messages received during go_front"

            # Verify at least some had positive linear.x
            forward_msgs = [t for t in listener.received_twists if t.linear.x > 0.0]
            assert len(forward_msgs) > 0, \
                "No forward velocity commands during go_front"

            # Verify the last message is a stop command (init_twist)
            last_msg = listener.received_twists[-1]
            assert last_msg.linear.x == 0.0, \
                f"Robot did not stop after go_front completed (linear.x={last_msg.linear.x})"

            # Test turn: rotate 45 degrees
            listener.clear()
            server.angular_z = 2.0

            turn_thread = threading.Thread(
                target=server.turn, args=(45.0,), daemon=True
            )
            turn_thread.start()
            turn_thread.join(timeout=10.0)

            # Small delay to ensure the final stop message is received
            time.sleep(0.2)

            # Verify cmd_vel messages were published during turn
            assert len(listener.received_twists) > 0, \
                "No cmd_vel messages received during turn"

            # Verify angular velocity was applied
            angular_msgs = [t for t in listener.received_twists if abs(t.angular.z) > 0.0]
            assert len(angular_msgs) > 0, \
                "No angular velocity commands during turn"

            # Verify proportional control: angular velocities should vary
            angular_values = [abs(t.angular.z) for t in angular_msgs]
            if len(angular_values) > 2:
                unique_values = set(round(v, 4) for v in angular_values)
                assert len(unique_values) > 1, \
                    "Angular velocities are constant - expected proportional control"

            # Verify the last message is a stop command
            last_msg = listener.received_twists[-1]
            assert last_msg.angular.z == 0.0, \
                "Robot did not stop after turn completed"

            executor.shutdown()

        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def test_source_code_oracle_checks(self):
        """Run the oracle static checks against the source file."""

        # Find the source file - check both locations
        code_file = None
        candidates = [
            Path(__file__).resolve().parent / "turtle_patrol_server.py",
            Path(__file__).resolve().parent / "task_007_autonomous_navigation" / "turtle_patrol_server.py",
        ]
        for c in candidates:
            if c.exists():
                code_file = c
                break
        assert code_file is not None, \
            f"Source file not found in any of: {candidates}"

        with open(code_file, 'r') as f:
            source = f.read()

        # Extract go_front function
        pattern = r"(?:async\s+)?def\s+go_front\s*\(.*?\):([\s\S]*?)(?=\n    def |\n    async def |\Z)"
        match = re.search(pattern, source)
        assert match, "go_front function not found"
        go_front_blk = match.group(1)

        # Check initial position recording
        has_initial_record = re.search(r"(initial|start|origin)_(x|y|pos)", go_front_blk)
        assert has_initial_record, "go_front must record initial position"

        # Check Euclidean distance formula
        has_distance_formula = re.search(r"(math\.sqrt|math\.hypot|\*\*\s*2\s*\+\s*.*?\*\*\s*2)", go_front_blk)
        assert has_distance_formula, "go_front must use Euclidean distance"

        # Extract turn function
        pattern = r"(?:async\s+)?def\s+turn\s*\(.*?\):([\s\S]*?)(?=\n    def |\n    async def |\Z)"
        match = re.search(pattern, source)
        assert match, "turn function not found"
        turn_blk = match.group(1)

        # Check atan2(sin, cos) pattern - use DOTALL so . matches newlines
        shortest_path_pattern = r"atan2\(.*?math\.sin\(.*?\).*?math\.cos\(.*?\).*?\)"
        assert re.search(shortest_path_pattern, turn_blk, re.DOTALL), \
            "turn must use atan2(sin(error), cos(error))"

        # Check proportional control
        p_control_pattern = r"angular\.z\s*=\s*.*?\*.*?(diff|error)"
        assert re.search(p_control_pattern, turn_blk, re.DOTALL), \
            "turn must use proportional control (Kp * error)"

        # Check safety
        assert "rclpy.ok()" in turn_blk or "timeout" in turn_blk.lower(), \
            "turn must have rclpy.ok() or timeout check"