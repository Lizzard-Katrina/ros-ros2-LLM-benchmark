#!/usr/bin/env python3
"""
Runtime test for the translated controller_manager_interface.py.
Launches a mock controller_manager server, then exercises the real
translated functions and asserts on concrete expected values.
"""
import os
import sys
import time
import subprocess
import signal
import pytest

# Ensure the package root (where controller_manager_interface.py lives) is importable
PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)


@pytest.fixture(scope='session', autouse=True)
def rclpy_context():
    """Initialize rclpy once for the entire test session."""
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='session')
def mock_server(rclpy_context):
    """Start the mock controller_manager node as a subprocess."""
    helper = os.path.join(PKG_ROOT, '_test_helper_node.py')
    env = os.environ.copy()
    proc = subprocess.Popen(
        [sys.executable, helper],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )
    # Give it time to start up
    time.sleep(4.0)
    yield proc
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


@pytest.fixture(autouse=True)
def reset_cmi_node():
    """Reset the module-level _node before and after each test."""
    import controller_manager_interface as cmi
    old_node = cmi._node
    if old_node is not None:
        old_node.destroy_node()
    cmi._node = None
    yield
    if cmi._node is not None:
        cmi._node.destroy_node()
        cmi._node = None


def test_start_stop_controllers(mock_server):
    """Test that start_stop_controllers calls the SwitchController service and returns True."""
    import controller_manager_interface as cmi

    result = cmi.start_stop_controllers(
        start_controllers=['joint_state_controller'],
        stop_controllers=['arm_controller']
    )
    assert result is True, "start_stop_controllers should return True when service responds ok"


def test_list_controllers(mock_server, capsys):
    """Test that list_controllers prints controller info."""
    import controller_manager_interface as cmi

    cmi.list_controllers()
    captured = capsys.readouterr()
    assert 'joint_state_controller' in captured.out, \
        f"Expected 'joint_state_controller' in output, got: {captured.out}"
    assert 'arm_controller' in captured.out, \
        f"Expected 'arm_controller' in output, got: {captured.out}"
    assert 'running' in captured.out, \
        f"Expected 'running' in output, got: {captured.out}"
    assert 'stopped' in captured.out, \
        f"Expected 'stopped' in output, got: {captured.out}"


def test_reload_libraries_no_restore(mock_server):
    """Test reload_libraries without restore returns True."""
    import controller_manager_interface as cmi

    result = cmi.reload_libraries(force_kill=True, restore=False)
    assert result is True, "reload_libraries should return True when service responds ok"


def test_reload_libraries_with_restore(mock_server, capsys):
    """Test reload_libraries with restore=True restores controllers."""
    import controller_manager_interface as cmi

    result = cmi.reload_libraries(force_kill=False, restore=True)
    assert result is True, "reload_libraries with restore should return True"
    captured = capsys.readouterr()
    assert 'restored' in captured.out.lower() or 'Restore' in captured.out, \
        f"Expected restore message in output, got: {captured.out}"


def test_load_controller(mock_server):
    """Test load_controller returns True for a successful load."""
    import controller_manager_interface as cmi

    result = cmi.load_controller('test_controller')
    assert result is True, "load_controller should return True when service responds ok"


def test_stop_controller(mock_server):
    """Test stop_controller delegates to start_stop_controllers."""
    import controller_manager_interface as cmi

    result = cmi.stop_controller('arm_controller')
    assert result is True, "stop_controller should return True when service responds ok"