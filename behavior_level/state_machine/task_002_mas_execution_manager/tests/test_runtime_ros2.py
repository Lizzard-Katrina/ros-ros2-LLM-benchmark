"""
Runtime test that verifies ScenarioStateBase works with a real ROS 2 node.
"""
import subprocess
import sys
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def test_scenario_state_base_say_publisher():
    """Test that ScenarioStateBase creates a latched /say publisher and can publish."""
    rclpy.init()
    received_msgs = []

    try:
        test_node = rclpy.create_node('test_scenario_listener')

        # Declare parameters that ScenarioStateBase will try to declare
        scenario_node = rclpy.create_node('scenario_test_node')
        # We don't pre-declare; the base class will declare them itself.

        # Import and instantiate the base class
        # We need smach available; provide a minimal mock if not installed
        import importlib
        try:
            import smach
        except ImportError:
            # Create a minimal smach mock
            import types
            smach = types.ModuleType('smach')

            class MockState:
                def __init__(self, outcomes=None, input_keys=None, output_keys=None):
                    pass

            smach.State = MockState
            sys.modules['smach'] = smach

        # Now import our module
        sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
        from scenario_state_base import ScenarioStateBase

        # Create the state with the scenario_node
        state = ScenarioStateBase(
            node=scenario_node,
            action_name='test_action',
            outcomes=['succeeded', 'failed']
        )

        # Verify attributes
        assert state.node is scenario_node
        assert state.action_name == 'test_action'
        assert state.sm_id == ''
        assert state.state_name == ''
        assert state.retry_count == 0
        assert state.executing is False
        assert state.succeeded is False

        # Verify parameters were declared and have default values
        ontology_url = scenario_node.get_parameter('ontology_url').value
        ontology_prefix = scenario_node.get_parameter('ontology_class_prefix').value
        assert ontology_url == ''
        assert ontology_prefix == ''

        # Subscribe to /say with transient local QoS to receive latched messages
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        def say_callback(msg):
            received_msgs.append(msg.data)

        sub = test_node.create_subscription(String, '/say', say_callback, qos_profile=latching_qos)

        # Give time for discovery
        for _ in range(20):
            rclpy.spin_once(test_node, timeout_sec=0.05)
            rclpy.spin_once(scenario_node, timeout_sec=0.05)

        # Publish via say method
        state.say('hello world')

        # Spin to receive
        deadline = time.time() + 3.0
        while time.time() < deadline and len(received_msgs) == 0:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            rclpy.spin_once(scenario_node, timeout_sec=0.05)

        assert len(received_msgs) > 0, "Did not receive any message on /say topic"
        assert received_msgs[0] == 'hello world'

    finally:
        try:
            test_node.destroy_node()
        except Exception:
            pass
        try:
            scenario_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


def test_scenario_state_base_parameters_custom():
    """Test that ScenarioStateBase reads custom parameter values."""
    rclpy.init()
    try:
        node = rclpy.create_node('param_test_node',
                                 parameter_overrides=[
                                     rclpy.parameter.Parameter('ontology_url',
                                                               rclpy.parameter.Parameter.Type.STRING,
                                                               'http://example.com/onto'),
                                     rclpy.parameter.Parameter('ontology_class_prefix',
                                                               rclpy.parameter.Parameter.Type.STRING,
                                                               'myprefix'),
                                 ])

        # Mock smach if needed
        try:
            import smach
        except ImportError:
            import types
            smach = types.ModuleType('smach')

            class MockState:
                def __init__(self, outcomes=None, input_keys=None, output_keys=None):
                    pass

            smach.State = MockState
            sys.modules['smach'] = smach

        sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
        from scenario_state_base import ScenarioStateBase

        state = ScenarioStateBase(
            node=node,
            action_name='pick',
            outcomes=['done']
        )

        assert state.ontology_url == 'http://example.com/onto'
        assert state.ontology_class_prefix == 'myprefix'

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()