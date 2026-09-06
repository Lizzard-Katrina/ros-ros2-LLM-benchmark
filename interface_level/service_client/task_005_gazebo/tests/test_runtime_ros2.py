"""
Runtime test for task_005_gazebo – exercises the GazeboInterfaceNode
by calling its spawn_entity service (SetBool) and verifying responses.
Also validates that gazebo_interface.py module loads and has the expected API.
"""
import subprocess
import time
import threading
import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize rclpy once for the whole module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def gazebo_node_process(ros_context):
    """Launch the gazebo_interface_node as a subprocess."""
    proc = subprocess.Popen(
        ['ros2', 'run', 'task_005_gazebo', 'gazebo_interface_node'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    # Give the node time to start up
    time.sleep(2.0)
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


def test_spawn_entity_service_true(gazebo_node_process):
    """Call spawn_entity with data=True, expect success=True."""
    node = rclpy.create_node('test_spawn_true')
    try:
        client = node.create_client(SetBool, 'spawn_entity')
        assert client.wait_for_service(timeout_sec=5.0), \
            "spawn_entity service not available"

        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done(), "Service call did not complete in time"
        response = future.result()
        assert response is not None, "No response received"
        assert response.success is True
        assert 'spawned successfully' in response.message.lower() or \
               'spawned' in response.message.lower()
    finally:
        node.destroy_node()


def test_spawn_entity_service_false(gazebo_node_process):
    """Call spawn_entity with data=False, expect success=False."""
    node = rclpy.create_node('test_spawn_false')
    try:
        client = node.create_client(SetBool, 'spawn_entity')
        assert client.wait_for_service(timeout_sec=5.0), \
            "spawn_entity service not available"

        request = SetBool.Request()
        request.data = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done(), "Service call did not complete in time"
        response = future.result()
        assert response is not None, "No response received"
        assert response.success is False
        assert 'declined' in response.message.lower()
    finally:
        node.destroy_node()


def test_gazebo_interface_module_api():
    """Verify that gazebo_interface.py has the expected function signatures."""
    import sys
    from pathlib import Path

    pkg_root = Path(__file__).resolve().parent
    if str(pkg_root) not in sys.path:
        sys.path.insert(0, str(pkg_root))

    import gazebo_interface

    # Check that the three main functions exist
    assert hasattr(gazebo_interface, 'spawn_sdf_model_client'), \
        "Missing spawn_sdf_model_client"
    assert hasattr(gazebo_interface, 'spawn_urdf_model_client'), \
        "Missing spawn_urdf_model_client"
    assert hasattr(gazebo_interface, 'set_model_configuration_client'), \
        "Missing set_model_configuration_client"

    # Check they are callable
    assert callable(gazebo_interface.spawn_sdf_model_client)
    assert callable(gazebo_interface.spawn_urdf_model_client)
    assert callable(gazebo_interface.set_model_configuration_client)


def test_gazebo_interface_imports():
    """Verify that gazebo_interface.py uses rclpy and geometry_msgs."""
    import sys
    from pathlib import Path

    pkg_root = Path(__file__).resolve().parent
    if str(pkg_root) not in sys.path:
        sys.path.insert(0, str(pkg_root))

    # Read the source and check for expected ROS2 patterns
    src = (Path(__file__).resolve().parent / 'gazebo_interface.py').read_text()
    assert 'rclpy' in src, "gazebo_interface.py should import rclpy"
    assert 'create_client' in src, "gazebo_interface.py should use create_client"
    assert 'call_async' in src, "gazebo_interface.py should use call_async"
    assert 'spin_until_future_complete' in src, \
        "gazebo_interface.py should use spin_until_future_complete"
    assert 'SpawnEntity' in src, "gazebo_interface.py should reference SpawnEntity"
    assert 'SetModelConfiguration' in src, \
        "gazebo_interface.py should reference SetModelConfiguration"
    assert 'geometry_msgs' in src, \
        "gazebo_interface.py should import geometry_msgs"