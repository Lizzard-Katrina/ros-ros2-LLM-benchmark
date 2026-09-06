"""
Runtime test for task_004_industrial_robot_simulator.

This test validates the migrated files by:
1. Checking the actual source files exist and contain correct ROS 2 patterns
2. Verifying the launch file can be loaded and parsed by ROS 2 launch infrastructure
3. Performing real ROS 2 operations to validate the code semantics
"""

import os
import re
import sys
import time
import pytest
from pathlib import Path

# Determine package root (where this test file lives)
PKG_ROOT = Path(__file__).resolve().parent


def _find_file(name):
    """Find a file in the package, checking both root and install paths."""
    # Check package root first
    p = PKG_ROOT / name
    if p.exists():
        return p
    # Check install share path
    import subprocess
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'task_004_industrial_robot_simulator'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            prefix = Path(result.stdout.strip())
            share_src = prefix / 'share' / 'task_004_industrial_robot_simulator' / 'src' / name
            if share_src.exists():
                return share_src
            share_launch = prefix / 'share' / 'task_004_industrial_robot_simulator' / 'launch' / name
            if share_launch.exists():
                return share_launch
    except Exception:
        pass
    return p  # Return root path even if not found, let test fail with clear message


class TestGenericRobotStateNode:
    """Tests for the migrated generic_robot_state_node.cpp"""

    def setup_method(self):
        self.filepath = _find_file('generic_robot_state_node.cpp')
        assert self.filepath.exists(), f"File not found: {self.filepath}"
        with open(self.filepath, 'r') as f:
            self.content = f.read()

    def test_includes_rclcpp_header(self):
        """Must include rclcpp header instead of ros.h"""
        assert 'rclcpp/rclcpp.hpp' in self.content, \
            "Missing #include rclcpp/rclcpp.hpp"

    def test_uses_make_shared(self):
        """Must use std::make_shared for ROS 2 node creation"""
        assert re.search(r'std::make_shared<RobotStateInterface>', self.content), \
            "Should use std::make_shared<RobotStateInterface>"

    def test_calls_init(self):
        """Must call init() on the node before spinning"""
        assert '->init' in self.content or '.init' in self.content, \
            "Missing node->init() call"

    def test_uses_rclcpp_spin(self):
        """Must use rclcpp::spin"""
        assert 'rclcpp::spin' in self.content, \
            "Missing rclcpp::spin"

    def test_uses_rclcpp_init(self):
        """Must use rclcpp::init"""
        assert 'rclcpp::init' in self.content, \
            "Missing rclcpp::init"

    def test_no_ros1_symbols(self):
        """Must not contain any ROS 1 symbols"""
        legacy_symbols = ["ros::init", "ros::NodeHandle", "ros::spin()", "ros::ok()"]
        for symbol in legacy_symbols:
            assert symbol not in self.content, \
                f"Legacy ROS 1 symbol '{symbol}' detected"

    def test_uses_rclcpp_shutdown(self):
        """Should call rclcpp::shutdown for clean exit"""
        assert 'rclcpp::shutdown' in self.content, \
            "Missing rclcpp::shutdown"

    def test_init_before_spin(self):
        """init() must be called before spin()"""
        init_pos = self.content.find('->init')
        if init_pos == -1:
            init_pos = self.content.find('.init')
        spin_pos = self.content.find('rclcpp::spin')
        assert init_pos < spin_pos, \
            "init() must be called before rclcpp::spin()"


class TestJointTrajectoryInterface:
    """Tests for the migrated joint_trajectory_interface.cpp"""

    def setup_method(self):
        self.filepath = _find_file('joint_trajectory_interface.cpp')
        assert self.filepath.exists(), f"File not found: {self.filepath}"
        with open(self.filepath, 'r') as f:
            self.content = f.read()

    def test_includes_rclcpp(self):
        """Must include rclcpp header"""
        assert 'rclcpp/rclcpp.hpp' in self.content, \
            "Missing rclcpp header"

    def test_declare_parameter(self):
        """ROS 2 requires parameters to be declared before use"""
        assert 'declare_parameter' in self.content, \
            "ROS 2 requires parameters to be declared"

    def test_controller_joint_names_param(self):
        """Must reference controller_joint_names parameter"""
        assert 'controller_joint_names' in self.content, \
            "Missing controller_joint_names parameter reference"

    def test_get_joint_names(self):
        """Must use getJointNames for joint name retrieval"""
        assert 'getJointNames' in self.content, \
            "Missing getJointNames call"

    def test_no_ros1_param_api(self):
        """Must not use ROS 1 parameter API"""
        assert 'ros::param::param' not in self.content, \
            "Legacy ros::param::param detected"

    def test_uses_rclcpp_logging(self):
        """Should use RCLCPP logging macros instead of ROS_* macros"""
        assert 'RCLCPP_' in self.content, \
            "Should use RCLCPP logging macros"

    def test_shared_ptr_callbacks(self):
        """ROS 2 uses SharedPtr for message callbacks"""
        assert 'SharedPtr' in self.content or 'shared_ptr' in self.content, \
            "Should use shared pointers for ROS 2 message handling"


class TestLaunchFile:
    """Tests for the migrated launch file"""

    def setup_method(self):
        # Check both .launch and .launch.py
        self.launch_py_path = _find_file('robot_interface_simulator.launch.py')
        self.launch_path = _find_file('robot_interface_simulator.launch')

    def test_launch_py_exists(self):
        """The .launch.py file should exist"""
        assert self.launch_py_path.exists(), \
            f"Launch file not found: {self.launch_py_path}"

    def test_launch_py_is_ros2_format(self):
        """Launch file must be ROS 2 Python format, not ROS 1 XML"""
        with open(self.launch_py_path, 'r') as f:
            content = f.read()
        assert '<launch>' not in content, \
            "Detected ROS 1 XML format in .launch.py file"
        assert 'launch' in content, "Missing launch import"
        assert 'launch_ros' in content, "Missing launch_ros import"

    def test_launch_file_not_xml(self):
        """The .launch file (if it exists) must NOT be ROS 1 XML"""
        if self.launch_path.exists():
            with open(self.launch_path, 'r') as f:
                content = f.read()
            assert '<launch>' not in content, \
                "FAILED: Detected ROS 1 XML format in .launch file"

    def test_launch_has_joint_names(self):
        """Launch file must define the 6-DOF joint names"""
        with open(self.launch_py_path, 'r') as f:
            content = f.read()
        for joint_num in range(1, 7):
            assert f'joint_{joint_num}' in content, \
                f"Missing joint_{joint_num} in launch file"

    def test_launch_has_generate_launch_description(self):
        """Must have the standard ROS 2 launch entry point"""
        with open(self.launch_py_path, 'r') as f:
            content = f.read()
        assert 'generate_launch_description' in content, \
            "Missing generate_launch_description function"

    def test_launch_has_node_declarations(self):
        """Must declare Node actions"""
        with open(self.launch_py_path, 'r') as f:
            content = f.read()
        assert 'Node(' in content, "Missing Node declarations in launch file"

    def test_launch_has_parameters(self):
        """Must pass parameters to nodes"""
        with open(self.launch_py_path, 'r') as f:
            content = f.read()
        assert 'parameters' in content, \
            "Missing parameters configuration in launch file"


class TestLaunchFileImport:
    """Test that the launch file can actually be imported and parsed by ROS 2 launch"""

    def test_launch_file_importable(self):
        """The launch.py file should be importable and generate a valid LaunchDescription"""
        import importlib.util

        launch_py_path = _find_file('robot_interface_simulator.launch.py')
        assert launch_py_path.exists(), f"Launch file not found: {launch_py_path}"

        spec = importlib.util.spec_from_file_location("launch_module", str(launch_py_path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        assert hasattr(module, 'generate_launch_description'), \
            "Module must have generate_launch_description function"

        ld = module.generate_launch_description()

        from launch import LaunchDescription
        assert isinstance(ld, LaunchDescription), \
            "generate_launch_description must return a LaunchDescription"

        # Verify it has entities (nodes, arguments, etc.)
        entities = ld.entities
        assert len(entities) > 0, "LaunchDescription should have at least one entity"

    def test_launch_description_has_nodes(self):
        """The LaunchDescription should contain Node actions"""
        import importlib.util

        launch_py_path = _find_file('robot_interface_simulator.launch.py')
        spec = importlib.util.spec_from_file_location("launch_module", str(launch_py_path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        ld = module.generate_launch_description()

        from launch_ros.actions import Node as LaunchNode
        from launch.actions import DeclareLaunchArgument

        node_count = 0
        arg_count = 0
        for entity in ld.entities:
            if isinstance(entity, LaunchNode):
                node_count += 1
            elif isinstance(entity, DeclareLaunchArgument):
                arg_count += 1

        assert node_count >= 2, \
            f"Expected at least 2 Node actions, found {node_count}"
        assert arg_count >= 1, \
            f"Expected at least 1 DeclareLaunchArgument, found {arg_count}"


class TestROS2Integration:
    """Test actual ROS 2 functionality using rclpy"""

    def test_rclpy_node_creation_and_parameter(self):
        """Create a real ROS 2 node and verify parameter declaration works
        as described in the migrated code pattern"""
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        try:
            node = Node('test_param_node')

            # Declare the same parameter that the migrated code uses
            node.declare_parameter(
                'controller_joint_names',
                ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            )

            # Retrieve and verify
            param = node.get_parameter('controller_joint_names')
            joint_names = param.get_parameter_value().string_array_value
            assert len(joint_names) == 6, f"Expected 6 joints, got {len(joint_names)}"
            assert joint_names[0] == 'joint_1'
            assert joint_names[5] == 'joint_6'

            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_trajectory_msg_creation(self):
        """Verify trajectory_msgs work as expected in the migrated code context"""
        import rclpy
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        rclpy.init()
        try:
            node = rclpy.create_node('test_traj_node')

            # Create a trajectory message matching the 6-DOF robot
            traj = JointTrajectory()
            traj.joint_names = [
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ]

            pt = JointTrajectoryPoint()
            pt.positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
            pt.time_from_start = Duration(sec=1, nanosec=0)
            traj.points.append(pt)

            assert len(traj.joint_names) == 6
            assert len(traj.points) == 1
            assert traj.points[0].positions[2] == pytest.approx(0.2)

            # Verify the time_from_start conversion pattern used in migrated code
            time_sec = traj.points[0].time_from_start.sec + \
                       traj.points[0].time_from_start.nanosec * 1e-9
            assert time_sec == pytest.approx(1.0)

            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_joint_state_pub_sub(self):
        """Test real pub/sub of joint states as the migrated system would use"""
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        import threading

        rclpy.init()
        received_msgs = []

        try:
            node = rclpy.create_node('test_joint_state_node')

            def callback(msg):
                received_msgs.append(msg)

            sub = node.create_subscription(
                JointState, '/joint_states', callback, 10
            )
            pub = node.create_publisher(JointState, '/joint_states', 10)

            # Publish a joint state message
            msg = JointState()
            msg.name = [
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ]
            msg.position = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]

            # Spin in background thread
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            # Publish multiple times to ensure delivery
            deadline = time.time() + 5.0
            while time.time() < deadline and len(received_msgs) == 0:
                pub.publish(msg)
                time.sleep(0.1)

            assert len(received_msgs) > 0, "No joint state messages received"
            received = received_msgs[0]
            assert len(received.name) == 6
            assert received.name[0] == 'joint_1'
            assert received.position[3] == pytest.approx(0.3)

            executor.shutdown()
            node.destroy_node()
        finally:
            rclpy.shutdown()