#!/usr/bin/env python3
"""
Runtime test for the task_004_simplified_pick_place package.
Verifies:
1. The ez_tools module can be imported and EZToolSet instantiated
2. The pose_factor scaling is correct (1000)
3. TF2 buffer/listener initialization works
4. Service client creation patterns
5. The fixItForGraspIt scaling logic
6. File content checks for correct ROS2 patterns
"""
import pytest
import rclpy
import tf2_ros
import re
from pathlib import Path


BASE_PATH = Path(__file__).resolve().parent


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestEZToolSetInitialization:
    """Test that EZToolSet can be created and has correct defaults."""

    def test_ez_toolset_creation(self):
        from task_004_simplified_pick_place.ez_tools import EZToolSet
        tools = EZToolSet()
        assert tools is not None

    def test_pose_factor_default(self):
        from task_004_simplified_pick_place.ez_tools import EZToolSet
        tools = EZToolSet()
        assert tools.pose_factor == 1000

    def test_tf2_buffer_initialization(self):
        """Verify TF2 buffer and listener can be set up on the toolset."""
        from task_004_simplified_pick_place.ez_tools import EZToolSet
        tools = EZToolSet()
        node = rclpy.create_node('test_tf2_init')
        try:
            tools.node = node
            tools.tf2_buffer = tf2_ros.Buffer()
            tools.tf2_listener = tf2_ros.TransformListener(tools.tf2_buffer, node)
            assert tools.tf2_buffer is not None
            assert tools.tf2_listener is not None
        finally:
            node.destroy_node()

    def test_node_creation_and_clients(self):
        """Verify that a node can create service clients with std_srvs as a stand-in."""
        from std_srvs.srv import Trigger
        node = rclpy.create_node('test_clients_node')
        try:
            client1 = node.create_client(Trigger, '/test_service_a')
            client2 = node.create_client(Trigger, '/test_service_b')
            assert client1 is not None
            assert client2 is not None
        finally:
            node.destroy_node()


class TestScalingLogic:
    """Test that the scaling factor is applied correctly in fixItForGraspIt."""

    def test_fix_it_for_graspit_world_frame(self):
        """When frame_id is 'world', scaling should be applied directly."""
        from task_004_simplified_pick_place.ez_tools import EZToolSet
        from geometry_msgs.msg import PoseStamped

        tools = EZToolSet()
        node = rclpy.create_node('test_scaling_node')
        try:
            tools.node = node
            tools.tf2_buffer = tf2_ros.Buffer()
            tools.tf2_listener = tf2_ros.TransformListener(tools.tf2_buffer, node)

            # Create a mock object with pose in world frame
            class MockObj:
                def __init__(self):
                    self.pose = PoseStamped()
                    self.pose.header.frame_id = "world"
                    self.pose.pose.position.x = 0.5
                    self.pose.pose.position.y = 0.3
                    self.pose.pose.position.z = 0.75
                    self.pose.pose.orientation.w = 1.0

            obj = MockObj()
            result = tools.fixItForGraspIt(obj, tools.pose_factor)

            assert result is not None
            # Verify scaling: 0.5 * 1000 = 500.0
            assert abs(result.position.x - 500.0) < 1e-6
            assert abs(result.position.y - 300.0) < 1e-6
            assert abs(result.position.z - 750.0) < 1e-6
            # Orientation should be unchanged
            assert abs(result.orientation.w - 1.0) < 1e-6
        finally:
            node.destroy_node()


class TestQuatHelpers:
    """Test the quaternion helper functions."""

    def test_euler2quat_identity(self):
        from task_004_simplified_pick_place.quat_helpers import euler2quat
        w, x, y, z = euler2quat(0, 0, 0)
        assert abs(w - 1.0) < 1e-6
        assert abs(x) < 1e-6
        assert abs(y) < 1e-6
        assert abs(z) < 1e-6

    def test_qmult_identity(self):
        from task_004_simplified_pick_place.quat_helpers import qmult
        result = qmult((1, 0, 0, 0), (1, 0, 0, 0))
        assert abs(result[0] - 1.0) < 1e-6
        assert abs(result[1]) < 1e-6
        assert abs(result[2]) < 1e-6
        assert abs(result[3]) < 1e-6


class TestFileContents:
    """Verify file contents match expected patterns (same as oracle tests)."""

    def test_ez_pnp2_has_rclpy_init(self):
        content = (BASE_PATH / "ez_pnp2.py").read_text()
        assert "rclpy.init" in content
        assert "rclpy.create_node" in content
        assert "import rospy" not in content

    def test_ez_pnp2_has_create_client(self):
        content = (BASE_PATH / "ez_pnp2.py").read_text()
        assert re.search(r"create_client\s*\(", content)
        assert "GraspPlanning" in content
        assert "GetPositionIK" in content

    def test_ez_tools_has_tf2(self):
        content = (BASE_PATH / "ez_tools.py").read_text()
        assert "tf2_ros.Buffer()" in content
        assert "lookup_transform" in content
        assert "rclpy.time.Time()" in content or "Duration" in content

    def test_ez_tools_has_async(self):
        content = (BASE_PATH / "ez_tools.py").read_text()
        assert "call_async" in content
        assert "spin_until_future_complete" in content

    def test_ez_tools_has_attach_detach(self):
        content = (BASE_PATH / "ez_tools.py").read_text()
        assert ".attach_object(" in content
        assert ".detach_object(" in content

    def test_ez_tools_has_scaling(self):
        content = (BASE_PATH / "ez_tools.py").read_text()
        assert "pose_factor = 1000" in content
        assert re.search(r"\*\s*self\.pose_factor", content) or re.search(r"\*\s*pose_factor", content)

    def test_test2_has_target_z(self):
        content = (BASE_PATH / "test2_ez_pnp2.py").read_text()
        assert re.search(r"graspit_target_object\s*=\s*[\"']Z[\"']", content)
        assert "rclpy.create_node" in content

    def test_integration_arm_move_group(self):
        content = (BASE_PATH / "test2_ez_pnp2.py").read_text()
        assert re.search(r"arm_move_group\s*=\s*[\"']arm[\"']", content)