#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Runtime test for task_002_custom_srv.
Launches the server node via subprocess (using bash -c to source the workspace),
then uses a test client to call the service and asserts on the actual response.
"""

import subprocess
import sys
import os
import time
import glob
import pytest


def _setup_env():
    """
    Source the colcon workspace so rosidl-generated Python packages are importable.
    """
    # Try importing directly first
    try:
        from task_002_custom_srv.srv import AddThreeInts  # noqa: F401
        return True
    except (ImportError, ModuleNotFoundError):
        pass

    # Find the workspace install directory
    ws_candidates = ['/ros2_ws', '/ros_ws', '/colcon_ws', '/workspace']
    install_dir = None

    for ws in ws_candidates:
        setup_bash = os.path.join(ws, 'install', 'setup.bash')
        pkg_dir = os.path.join(ws, 'install', 'task_002_custom_srv')
        if os.path.isdir(pkg_dir):
            install_dir = os.path.join(ws, 'install')
            break

    if install_dir is None:
        return False

    pkg_install = os.path.join(install_dir, 'task_002_custom_srv')

    # Add the package to AMENT_PREFIX_PATH
    ament = os.environ.get('AMENT_PREFIX_PATH', '')
    if pkg_install not in ament:
        os.environ['AMENT_PREFIX_PATH'] = pkg_install + (':' + ament if ament else '')

    # Find all Python path directories under the package install
    python_dirs = set()
    for root, dirs, files in os.walk(pkg_install):
        basename = os.path.basename(root)
        if basename in ('site-packages', 'dist-packages'):
            python_dirs.add(root)
        # Also check if there's a task_002_custom_srv directory with srv subdir
        task_dir = os.path.join(root, 'task_002_custom_srv')
        if os.path.isdir(task_dir) and os.path.isdir(os.path.join(task_dir, 'srv')):
            python_dirs.add(root)

    # Also search in the build directory for rosidl_generator_py output
    for ws in ws_candidates:
        build_py = os.path.join(ws, 'build', 'task_002_custom_srv', 'rosidl_generator_py')
        if os.path.isdir(build_py):
            python_dirs.add(build_py)

    for pp in sorted(python_dirs):
        if pp not in sys.path:
            sys.path.insert(0, pp)

    # Update PYTHONPATH for subprocesses
    pypath = os.environ.get('PYTHONPATH', '')
    for pp in sorted(python_dirs):
        if pp not in pypath:
            os.environ['PYTHONPATH'] = pp + (':' + pypath if pypath else '')
            pypath = os.environ['PYTHONPATH']

    # Also need to set LD_LIBRARY_PATH for the C extension modules
    lib_dirs = set()
    for root, dirs, files in os.walk(pkg_install):
        for f in files:
            if f.endswith('.so'):
                lib_dirs.add(root)
    for ws in ws_candidates:
        build_dir = os.path.join(ws, 'build', 'task_002_custom_srv')
        if os.path.isdir(build_dir):
            for root, dirs, files in os.walk(build_dir):
                for f in files:
                    if f.endswith('.so'):
                        lib_dirs.add(root)

    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    for ld in sorted(lib_dirs):
        if ld not in ld_path:
            os.environ['LD_LIBRARY_PATH'] = ld + (':' + ld_path if ld_path else '')
            ld_path = os.environ['LD_LIBRARY_PATH']

    # Try import again
    try:
        from task_002_custom_srv.srv import AddThreeInts  # noqa: F401
        return True
    except (ImportError, ModuleNotFoundError):
        pass

    return False


def _find_workspace():
    """Find the ROS2 workspace root."""
    for ws in ['/ros2_ws', '/ros_ws', '/colcon_ws', '/workspace']:
        if os.path.isdir(os.path.join(ws, 'install', 'task_002_custom_srv')):
            return ws
    return None


def _get_sourced_env():
    """Get environment variables from sourcing the workspace setup.bash."""
    ws = _find_workspace()
    if ws is None:
        return os.environ.copy()

    setup_bash = os.path.join(ws, 'install', 'setup.bash')
    if not os.path.isfile(setup_bash):
        return os.environ.copy()

    try:
        result = subprocess.run(
            ['bash', '-c', f'source {setup_bash} && env'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            env = {}
            for line in result.stdout.splitlines():
                if '=' in line:
                    key, _, val = line.partition('=')
                    env[key] = val
            return env
    except Exception:
        pass

    return os.environ.copy()


# Try direct import setup
_direct_import_ok = _setup_env()


def _get_subprocess_env():
    """Return an env dict suitable for launching ros2 run subprocesses."""
    return _get_sourced_env()


def test_add_three_ints_service():
    """Launch the real server node, call the service, and verify the sum."""
    ws = _find_workspace()
    assert ws is not None, "Cannot find ROS2 workspace"

    env = _get_sourced_env()

    # Launch server via bash sourcing the workspace
    setup_bash = os.path.join(ws, 'install', 'setup.bash')
    server_cmd = f'source {setup_bash} && ros2 run task_002_custom_srv ros_server.py'

    server_proc = subprocess.Popen(
        ['bash', '-c', server_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )

    try:
        # Give the server time to start
        time.sleep(3.0)

        # Check server is still running
        assert server_proc.poll() is None, \
            f"Server process exited early with code {server_proc.returncode}. " \
            f"stderr: {server_proc.stderr.read().decode()}"

        # Use a subprocess client to call the service and verify
        # This avoids needing the import in the test process itself
        client_script = f'''
import sys
import rclpy
from rclpy.node import Node
from task_002_custom_srv.srv import AddThreeInts

rclpy.init()
node = Node("test_client_node")
client = node.create_client(AddThreeInts, "add_three_ints")
assert client.wait_for_service(timeout_sec=10.0), "Service not available"

# Test 1: 1+2+3=6
req = AddThreeInts.Request()
req.a, req.b, req.c = 1, 2, 3
fut = client.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
assert fut.done() and fut.result() is not None
assert fut.result().sum == 6, f"Expected 6, got {{fut.result().sum}}"

# Test 2: 10+20+30=60
req2 = AddThreeInts.Request()
req2.a, req2.b, req2.c = 10, 20, 30
fut2 = client.call_async(req2)
rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
assert fut2.done() and fut2.result() is not None
assert fut2.result().sum == 60, f"Expected 60, got {{fut2.result().sum}}"

# Test 3: -5+0+5=0
req3 = AddThreeInts.Request()
req3.a, req3.b, req3.c = -5, 0, 5
fut3 = client.call_async(req3)
rclpy.spin_until_future_complete(node, fut3, timeout_sec=5.0)
assert fut3.done() and fut3.result() is not None
assert fut3.result().sum == 0, f"Expected 0, got {{fut3.result().sum}}"

node.destroy_node()
rclpy.shutdown()
print("ALL_TESTS_PASSED")
'''
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(client_script)
            client_script_path = f.name
        try:
            client_cmd = f'source {setup_bash} && python3 {client_script_path}'
            client_result = subprocess.run(
                ['bash', '-c', client_cmd],
                capture_output=True, text=True, timeout=30, env=env,
            )
        finally:
            os.unlink(client_script_path)

        assert client_result.returncode == 0, \
            f"Client failed: stdout={client_result.stdout}, stderr={client_result.stderr}"
        assert "ALL_TESTS_PASSED" in client_result.stdout, \
            f"Tests did not pass: stdout={client_result.stdout}, stderr={client_result.stderr}"

    finally:
        server_proc.terminate()
        try:
            server_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_proc.kill()
            server_proc.wait()


def test_client_node_structure():
    """Verify the client module exists and has expected structure."""
    from pathlib import Path

    client_path = Path(__file__).parent / "ros_client.py"
    assert client_path.exists(), f"ros_client.py not found at {client_path}"

    content = client_path.read_text()
    assert 'AddThreeIntsClient' in content, "ros_client.py must define AddThreeIntsClient"
    assert 'client_node' in content, "ros_client.py must define client_node function"
    assert 'AddThreeInts' in content, "ros_client.py must use AddThreeInts service"
    assert 'create_client' in content, "ros_client.py must use create_client"


def test_server_node_structure():
    """Verify the server module exists and has expected structure."""
    from pathlib import Path

    server_path = Path(__file__).parent / "ros_server.py"
    assert server_path.exists(), f"ros_server.py not found at {server_path}"

    content = server_path.read_text()
    assert 'AddThreeIntsServer' in content, "ros_server.py must define AddThreeIntsServer"
    assert 'server_node' in content, "ros_server.py must define server_node function"
    assert 'AddThreeInts' in content, "ros_server.py must use AddThreeInts service"
    assert 'create_service' in content, "ros_server.py must use create_service"


def test_service_definition():
    """Verify the service definition file exists and has correct fields."""
    from pathlib import Path

    srv_path = Path(__file__).parent / "srv" / "AddThreeInts.srv"
    assert srv_path.exists(), f"AddThreeInts.srv not found at {srv_path}"

    content = srv_path.read_text()
    assert 'int64 a' in content
    assert 'int64 b' in content
    assert 'int64 c' in content
    assert 'int64 sum' in content
    assert '---' in content