"""
Runtime test for the migrated pick_and_place.py.

This test validates the structural and semantic correctness of the migrated
ROS2 pick-and-place script by:
1. Mocking the hardware-dependent Interbotix modules
2. Actually importing and running the translated main() function
3. Asserting that the correct ROS2 patterns were used (shared node, TF naming, etc.)
"""
import sys
import types
import pytest
import threading


# ---------------------------------------------------------------------------
# Build mock modules that mimic the Interbotix ROS2 Python API surface
# so we can import and run pick_and_place.py without real hardware.
# ---------------------------------------------------------------------------

_call_log = []


class _MockArm:
    def set_ee_pose_components(self, **kwargs):
        _call_log.append(('arm.set_ee_pose_components', kwargs))

    def set_ee_cartesian_trajectory(self, **kwargs):
        _call_log.append(('arm.set_ee_cartesian_trajectory', kwargs))

    def go_to_sleep_pose(self):
        _call_log.append(('arm.go_to_sleep_pose', {}))


class _MockGripper:
    def open(self):
        _call_log.append(('gripper.open', {}))

    def close(self):
        _call_log.append(('gripper.close', {}))


class _MockDxl:
    def robot_set_motor_registers(self, *args, **kwargs):
        _call_log.append(('dxl.robot_set_motor_registers', args))


_shared_nodes = []


class MockInterbotixManipulatorXS:
    def __init__(self, robot_model=None, moving_time=None, accel_time=None,
                 gripper_pressure=None, node=None, **kwargs):
        _call_log.append(('InterbotixManipulatorXS.__init__', {
            'robot_model': robot_model,
            'node': node,
            'moving_time': moving_time,
            'accel_time': accel_time,
        }))
        if node is not None:
            _shared_nodes.append(('bot', node))
        self.arm = _MockArm()
        self.gripper = _MockGripper()
        self.dxl = _MockDxl()


class MockInterbotixArmTagInterface:
    def __init__(self, node=None, **kwargs):
        _call_log.append(('InterbotixArmTagInterface.__init__', {'node': node}))
        if node is not None:
            _shared_nodes.append(('armtag', node))

    def find_ref_to_arm_base_transform(self):
        _call_log.append(('armtag.find_ref_to_arm_base_transform', {}))


class MockInterbotixPointCloudInterface:
    def __init__(self, node=None, **kwargs):
        _call_log.append(('InterbotixPointCloudInterface.__init__', {'node': node}))
        if node is not None:
            _shared_nodes.append(('pcl', node))

    def get_cluster_positions(self, ref_frame=None, sort_axis=None, reverse=False, **kwargs):
        _call_log.append(('pcl.get_cluster_positions', {
            'ref_frame': ref_frame,
            'sort_axis': sort_axis,
            'reverse': reverse,
        }))
        # Return two fake clusters so the loop body executes
        clusters = [
            {"position": (0.25, -0.05, 0.02), "color": (255, 0, 0)},
            {"position": (0.20, 0.05, 0.03), "color": (0, 0, 255)},
        ]
        return True, clusters


# ---------------------------------------------------------------------------
# Patch rclpy so we don't need a real ROS2 environment
# ---------------------------------------------------------------------------

class _FakeNode:
    """Minimal stand-in for rclpy.node.Node."""
    def __init__(self, name):
        self.name = name

    def destroy_node(self):
        _call_log.append(('node.destroy_node', {}))

    def get_logger(self):
        class _L:
            def info(self, msg): pass
            def warn(self, msg): pass
            def error(self, msg): pass
        return _L()


_rclpy_init_called = False
_rclpy_shutdown_called = False
_created_node = None


def _fake_rclpy_init(*args, **kwargs):
    global _rclpy_init_called
    _rclpy_init_called = True
    _call_log.append(('rclpy.init', {}))


def _fake_rclpy_shutdown(*args, **kwargs):
    global _rclpy_shutdown_called
    _rclpy_shutdown_called = True
    _call_log.append(('rclpy.shutdown', {}))


def _fake_create_node(name, **kwargs):
    global _created_node
    _created_node = _FakeNode(name)
    _call_log.append(('rclpy.create_node', {'name': name}))
    return _created_node


def _install_mocks():
    """Install all mock modules into sys.modules before importing the target."""
    global _rclpy_init_called, _rclpy_shutdown_called, _created_node
    _rclpy_init_called = False
    _rclpy_shutdown_called = False
    _created_node = None
    _call_log.clear()
    _shared_nodes.clear()

    # Mock rclpy
    fake_rclpy = types.ModuleType('rclpy')
    fake_rclpy.init = _fake_rclpy_init
    fake_rclpy.shutdown = _fake_rclpy_shutdown
    fake_rclpy.create_node = _fake_create_node
    fake_rclpy_node = types.ModuleType('rclpy.node')
    fake_rclpy.node = fake_rclpy_node
    sys.modules['rclpy'] = fake_rclpy
    sys.modules['rclpy.node'] = fake_rclpy_node

    # Mock interbotix_xs_modules.xs_robot.arm
    mod_xs = types.ModuleType('interbotix_xs_modules')
    mod_xs_robot = types.ModuleType('interbotix_xs_modules.xs_robot')
    mod_xs_robot_arm = types.ModuleType('interbotix_xs_modules.xs_robot.arm')
    mod_xs_robot_arm.InterbotixManipulatorXS = MockInterbotixManipulatorXS
    mod_xs.xs_robot = mod_xs_robot
    mod_xs_robot.arm = mod_xs_robot_arm
    sys.modules['interbotix_xs_modules'] = mod_xs
    sys.modules['interbotix_xs_modules.xs_robot'] = mod_xs_robot
    sys.modules['interbotix_xs_modules.xs_robot.arm'] = mod_xs_robot_arm

    # Also mock the old-style import path in case it's used
    mod_xs_arm_old = types.ModuleType('interbotix_xs_modules.arm')
    mod_xs_arm_old.InterbotixManipulatorXS = MockInterbotixManipulatorXS
    sys.modules['interbotix_xs_modules.arm'] = mod_xs_arm_old

    # Mock interbotix_perception_modules.armtag
    mod_perc = types.ModuleType('interbotix_perception_modules')
    mod_perc_armtag = types.ModuleType('interbotix_perception_modules.armtag')
    mod_perc_armtag.InterbotixArmTagInterface = MockInterbotixArmTagInterface
    mod_perc_pcl = types.ModuleType('interbotix_perception_modules.pointcloud')
    mod_perc_pcl.InterbotixPointCloudInterface = MockInterbotixPointCloudInterface
    mod_perc.armtag = mod_perc_armtag
    mod_perc.pointcloud = mod_perc_pcl
    sys.modules['interbotix_perception_modules'] = mod_perc
    sys.modules['interbotix_perception_modules.armtag'] = mod_perc_armtag
    sys.modules['interbotix_perception_modules.pointcloud'] = mod_perc_pcl


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _run_main():
    """Import and run the translated pick_and_place main()."""
    # Remove cached module so it re-imports with our mocks
    for key in list(sys.modules.keys()):
        if 'pick_and_place' in key:
            del sys.modules[key]

    _install_mocks()

    # Import the actual translated file
    from task_005_interbotix_demo.pick_and_place import main as pick_place_main
    pick_place_main()


def test_rclpy_lifecycle():
    """rclpy.init() and rclpy.shutdown() (or node.destroy_node) must be called."""
    _run_main()
    call_names = [c[0] for c in _call_log]
    assert 'rclpy.init' in call_names, "rclpy.init() was never called"
    assert 'rclpy.shutdown' in call_names or 'node.destroy_node' in call_names, \
        "Neither rclpy.shutdown() nor node.destroy_node() was called"


def test_shared_node_instance():
    """All three interfaces must receive the same node object."""
    _run_main()
    assert len(_shared_nodes) == 3, \
        f"Expected 3 interfaces to receive a node, got {len(_shared_nodes)}"
    nodes_set = set(id(n) for _, n in _shared_nodes)
    assert len(nodes_set) == 1, \
        "Not all interfaces share the same node instance"


def test_robot_model_is_wx200():
    """InterbotixManipulatorXS must be constructed with robot_model='wx200'."""
    _run_main()
    bot_init = [c for c in _call_log if c[0] == 'InterbotixManipulatorXS.__init__']
    assert len(bot_init) == 1
    assert bot_init[0][1]['robot_model'] == 'wx200'


def test_ref_frame_no_leading_slash():
    """get_cluster_positions must use ref_frame='wx200/base_link' (no leading slash)."""
    _run_main()
    pcl_calls = [c for c in _call_log if c[0] == 'pcl.get_cluster_positions']
    assert len(pcl_calls) == 1
    ref = pcl_calls[0][1]['ref_frame']
    assert ref == 'wx200/base_link', f"Expected 'wx200/base_link', got '{ref}'"
    assert not ref.startswith('/'), "ref_frame must not start with '/'"


def test_cluster_loop_executes():
    """The pick-and-place loop must execute for each cluster (we provided 2)."""
    _run_main()
    gripper_close_count = sum(1 for c in _call_log if c[0] == 'gripper.close')
    assert gripper_close_count == 2, \
        f"Expected gripper.close() called 2 times (one per cluster), got {gripper_close_count}"


def test_sleep_pose_called():
    """go_to_sleep_pose must be called at the end."""
    _run_main()
    call_names = [c[0] for c in _call_log]
    assert 'arm.go_to_sleep_pose' in call_names, "go_to_sleep_pose() was never called"


def test_no_rospy_import():
    """The translated file must not import rospy."""
    import importlib
    _install_mocks()
    for key in list(sys.modules.keys()):
        if 'pick_and_place' in key:
            del sys.modules[key]
    import inspect
    from task_005_interbotix_demo import pick_and_place
    src = inspect.getsource(pick_and_place)
    assert 'import rospy' not in src, "Legacy rospy import found in translated file"
    assert 'rospy.init_node' not in src, "Legacy rospy.init_node found in translated file"


def test_moving_time_param():
    """InterbotixManipulatorXS must receive moving_time=1.5."""
    _run_main()
    bot_init = [c for c in _call_log if c[0] == 'InterbotixManipulatorXS.__init__']
    assert len(bot_init) == 1
    assert bot_init[0][1]['moving_time'] == 1.5, \
        f"Expected moving_time=1.5, got {bot_init[0][1]['moving_time']}"