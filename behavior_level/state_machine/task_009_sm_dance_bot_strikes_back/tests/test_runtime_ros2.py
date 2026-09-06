"""
Runtime test for the OdomTracker ROS2 migration.
Launches the odom_tracker_node, publishes odometry messages, and verifies
that the node publishes path messages on the expected topics.
"""
import subprocess
import time
import pytest

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion


def make_odom_msg(x, y, frame_id="odom"):
    """Create an Odometry message at position (x, y)."""
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return msg


class PathCollector(Node):
    """Helper node that subscribes to path topics and publishes odom."""

    def __init__(self):
        super().__init__("test_path_collector")
        self.received_paths = []
        self.received_stacked_paths = []

        self.path_sub = self.create_subscription(
            Path, "odom_tracker_path", self._path_cb, 10
        )
        self.stacked_path_sub = self.create_subscription(
            Path, "odom_tracker_stacked_path", self._stacked_path_cb, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

    def _path_cb(self, msg):
        self.received_paths.append(msg)

    def _stacked_path_cb(self, msg):
        self.received_stacked_paths.append(msg)

    def publish_odom(self, x, y):
        msg = make_odom_msg(x, y)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(msg)


def test_odom_tracker_publishes_path():
    """
    Test that the odom_tracker_node:
    1. Starts without error
    2. Subscribes to 'odom' topic
    3. Publishes recorded path on 'odom_tracker_path'
    4. Path contains the expected number of poses
    """
    rclpy.init()
    proc = None
    collector = None
    try:
        # Launch the actual odom_tracker_node executable from the built package
        proc = subprocess.Popen(
            ["ros2", "run", "task_009_sm_dance_bot_strikes_back", "odom_tracker_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the node time to start
        time.sleep(2.0)

        collector = PathCollector()

        # Publish a sequence of odometry messages with enough distance between them
        # The default record_point_distance_threshold is 0.005 m
        positions = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
        ]

        deadline = time.time() + 10.0  # 10 second timeout
        pos_idx = 0
        publish_interval = 0.3  # seconds between publishes
        last_publish = 0.0

        while time.time() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.05)

            now = time.time()
            if pos_idx < len(positions) and (now - last_publish) >= publish_interval:
                x, y = positions[pos_idx]
                collector.publish_odom(x, y)
                pos_idx += 1
                last_publish = now

            # Check if we've received enough path messages with poses
            if collector.received_paths:
                latest = collector.received_paths[-1]
                if len(latest.poses) >= 3:
                    break

        # Assertions
        assert len(collector.received_paths) > 0, (
            "No path messages received on 'odom_tracker_path'"
        )

        latest_path = collector.received_paths[-1]
        assert len(latest_path.poses) >= 3, (
            f"Expected at least 3 poses in path, got {len(latest_path.poses)}"
        )

        # Verify the path frame_id matches expected odom frame
        assert latest_path.header.frame_id == "odom", (
            f"Expected frame_id 'odom', got '{latest_path.header.frame_id}'"
        )

        # Verify poses are roughly at the positions we published
        first_pose = latest_path.poses[0].pose.position
        assert abs(first_pose.x - 0.0) < 0.1, (
            f"First pose x should be ~0.0, got {first_pose.x}"
        )

        # Check stacked path topic was also published
        assert len(collector.received_stacked_paths) > 0, (
            "No messages received on 'odom_tracker_stacked_path'"
        )

    finally:
        if collector is not None:
            collector.destroy_node()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        rclpy.shutdown()


def test_odom_tracker_source_file_exists():
    """Verify the source file exists at the expected location (sanity check)."""
    from pathlib import Path as FilePath
    cpp_file = FilePath(__file__).resolve().parent / "odom_tracker.cpp"
    assert cpp_file.exists(), f"Source file not found at {cpp_file}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])