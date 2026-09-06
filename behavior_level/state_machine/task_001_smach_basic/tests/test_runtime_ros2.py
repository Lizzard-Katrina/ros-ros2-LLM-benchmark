#!/usr/bin/env python3
"""
Runtime test for the migrated ServiceState.
Tests actual ROS2 service call through the ServiceState execute method.
"""
import pytest
import threading
import time
import sys
import os

# Ensure the package root is on the path so service_state.py can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def test_service_state_succeeds():
    """Test that ServiceState can call a real ROS2 service and return 'succeeded'."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    from task_001_smach_basic.smach_minimal import UserData

    rclpy.init()
    node = None
    client_node = None
    try:
        node = Node('test_service_state_node')

        # Create a real service on this node
        def handle_set_bool(request, response):
            response.success = request.data
            response.message = 'ok' if request.data else 'not ok'
            return response

        srv = node.create_service(SetBool, '/test_ss_set_bool', handle_set_bool)

        # Spin in background so the service can respond
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # Give the service a moment to be discoverable
        time.sleep(0.5)

        # Now create a second node for the ServiceState client
        client_node = Node('test_service_state_client')

        # Import ServiceState
        from service_state import ServiceState

        # Create the ServiceState
        ss = ServiceState(
            client_node,
            service_name='/test_ss_set_bool',
            service_spec=SetBool,
            request=SetBool.Request(data=True),
            response_slots=['success', 'message'],
            output_keys=['success', 'message'],
        )

        # Create userdata
        ud = UserData()

        # Execute the state (this will call the service)
        outcome = ss.execute(ud)

        assert outcome == 'succeeded', f"Expected 'succeeded', got '{outcome}'"
        assert ud.success == True, f"Expected success=True, got {ud.success}"
        assert ud.message == 'ok', f"Expected message='ok', got {ud.message}"

    finally:
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_service_state_with_request_slots():
    """Test that ServiceState correctly maps userdata slots to request fields."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    from task_001_smach_basic.smach_minimal import UserData

    if not rclpy.ok():
        rclpy.init()

    node = None
    client_node = None
    try:
        node = Node('test_ss_slots_node')

        def handle_set_bool(request, response):
            response.success = request.data
            response.message = 'slot_test'
            return response

        srv = node.create_service(SetBool, '/test_ss_slots_svc', handle_set_bool)

        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        time.sleep(0.5)

        client_node = Node('test_ss_slots_client')

        from service_state import ServiceState

        ss = ServiceState(
            client_node,
            service_name='/test_ss_slots_svc',
            service_spec=SetBool,
            request_slots=['data'],
            input_keys=['data'],
            response_key='response',
            output_keys=['response'],
        )

        ud = UserData()
        ud.data = False

        outcome = ss.execute(ud)

        assert outcome == 'succeeded', f"Expected 'succeeded', got '{outcome}'"
        assert ud.response.success == False, f"Expected response.success=False"
        assert ud.response.message == 'slot_test'

    finally:
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_service_state_with_request_cb():
    """Test that ServiceState correctly invokes the request callback."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    from task_001_smach_basic.smach_minimal import UserData

    if not rclpy.ok():
        rclpy.init()

    node = None
    client_node = None
    try:
        node = Node('test_ss_cb_node')

        def handle_set_bool(request, response):
            response.success = request.data
            response.message = 'cb_test'
            return response

        srv = node.create_service(SetBool, '/test_ss_cb_svc', handle_set_bool)

        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        time.sleep(0.5)

        client_node = Node('test_ss_cb_client')

        from service_state import ServiceState

        def my_request_cb(ud, request):
            request.data = True
            return request

        ss = ServiceState(
            client_node,
            service_name='/test_ss_cb_svc',
            service_spec=SetBool,
            request_cb=my_request_cb,
            response_slots=['success', 'message'],
            output_keys=['success', 'message'],
        )

        ud = UserData()
        outcome = ss.execute(ud)

        assert outcome == 'succeeded', f"Expected 'succeeded', got '{outcome}'"
        assert ud.success == True
        assert ud.message == 'cb_test'

    finally:
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short', '-x'])