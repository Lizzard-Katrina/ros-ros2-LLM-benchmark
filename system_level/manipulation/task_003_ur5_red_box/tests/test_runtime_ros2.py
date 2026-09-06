#!/usr/bin/env python3
"""
Runtime test for task_003_ur5_red_box.
Launches the service server, then uses a client to call it and verify the response.
"""

import pytest
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_service_call_roundtrip():
    """
    Launch the service server as a subprocess, create a client in-process,
    call the service, and verify the response content.
    """
    # Launch the server
    server_proc = subprocess.Popen(
        ['python3', 'scripts/set_joint_states_service.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    node = None
    try:
        # Give the server time to start
        time.sleep(3.0)

        from task_003_ur5_red_box.srv import SetJointStates

        node = rclpy.create_node('test_client_node')
        client = node.create_client(SetJointStates, 'set_joint_states')

        # Wait for service availability
        ready = client.wait_for_service(timeout_sec=8.0)
        assert ready, "Service 'set_joint_states' did not become available in time."

        # Build request
        request = SetJointStates.Request()
        request.forearm_0 = Float32(data=1.0)
        request.forearm_1 = Float32(data=2.0)
        request.arm_0 = Float32(data=3.0)
        request.arm_1 = Float32(data=4.0)

        # Async call
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        result = future.result()
        assert result is not None, "Service call returned None (timeout or failure)."
        assert result.success is True, f"Expected success=True, got {result.success}"
        assert '1.0' in result.message, f"Expected '1.0' in message, got: {result.message}"
        assert '2.0' in result.message, f"Expected '2.0' in message, got: {result.message}"
        assert '3.0' in result.message, f"Expected '3.0' in message, got: {result.message}"
        assert '4.0' in result.message, f"Expected '4.0' in message, got: {result.message}"

    finally:
        if node is not None:
            node.destroy_node()
        server_proc.terminate()
        server_proc.wait(timeout=5)


def test_oracle_service_server_migration():
    """Verify service server file structure matches oracle expectations."""
    import re
    from pathlib import Path

    service_file = Path(__file__).resolve().parent / 'set_joint_states_service.py'
    content = service_file.read_text()

    assert re.search(r"def\s+\w+\(self,\s*request,\s*response\):", content) or \
           re.search(r"def\s+\w+\(request,\s*response\):", content), \
           "Service callback must accept (request, response) arguments."
    assert "return response" in content
    assert "rclpy.init" in content
    assert "create_service" in content


def test_oracle_service_client_async_integrity():
    """Verify client file uses async patterns."""
    from pathlib import Path

    client_file = Path(__file__).resolve().parent / 'set_joint_states_client.py'
    content = client_file.read_text()

    assert "call_async" in content
    assert "spin_until_future_complete" in content or ".result()" in content
    assert "rospy" not in content.lower()


def test_oracle_cmakelists_rosidl_integrity():
    """Verify CMakeLists.txt uses ROS2 build patterns."""
    import re
    from pathlib import Path

    cmake_file = Path(__file__).resolve().parent / 'CMakeLists.txt'
    content = cmake_file.read_text()

    assert "rosidl_generate_interfaces" in content
    assert re.search(r"DESTINATION\s+lib/\${PROJECT_NAME}", content)
    assert "ament_package()" in content
    for macro in ["catkin_package", "add_service_files", "generate_messages"]:
        assert macro not in content


def test_oracle_package_xml_modern_format():
    """Verify package.xml follows format 3."""
    from pathlib import Path

    pkg_file = Path(__file__).resolve().parent / 'package.xml'
    content = pkg_file.read_text()

    assert 'format="3"' in content
    assert "<build_type>ament_cmake</build_type>" in content
    assert "rosidl_interface_packages" in content
    assert "rosidl_default_generators" in content
    assert "rclpy" in content
    assert "rospy" not in content


def test_oracle_no_rospy_anywhere():
    """Ensure no ROS1 remnants."""
    from pathlib import Path

    files = [
        'set_joint_states_service.py',
        'set_joint_states_client.py',
        'CMakeLists.txt',
        'package.xml',
    ]
    base = Path(__file__).resolve().parent
    all_content = ""
    for f in files:
        fp = base / f
        if fp.exists():
            all_content += fp.read_text()
    all_lower = all_content.lower()
    assert "rospy" not in all_lower
    assert "catkin" not in all_lower