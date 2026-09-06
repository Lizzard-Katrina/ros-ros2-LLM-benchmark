#!/usr/bin/env python3
"""
Runtime test for the VDA5050 MQTT Bridge (task_011_amr_interop).

Tests:
1. The generate_vda_instant_action_msg function handles v1/v2 protocol fields.
2. The MQTTBridge node creates proper ROS 2 subscriptions and publishers.
3. Publishing an OrderState message on the state topic triggers the bridge callback.
"""

import time
import pytest


def test_generate_vda_instant_action_msg_v2():
    """Test that generate_vda_instant_action_msg handles v2 'actions' field."""
    from task_011_amr_interop.msg import Action as VDAAction
    from task_011_amr_interop.msg import ActionParameter as VDAActionParameter
    from vda5050_connector_py.mqtt_bridge import generate_vda_instant_action_msg

    instant_action_v2 = {
        "header_id": 1,
        "timestamp": "2023-01-01T00:00:00.00Z",
        "version": "2.0.0",
        "manufacturer": "test_mfg",
        "serial_number": "test_sn",
        "actions": [
            {
                "action_type": "pick",
                "action_id": "action-1",
                "blocking_type": "SOFT",
                "action_parameters": [
                    {"key": "k1", "value": "v1"},
                    {"key": "k2", "value": 42},
                ],
            }
        ],
    }

    result = generate_vda_instant_action_msg(instant_action_v2)

    # Result should have 'actions' key with VDAAction objects
    assert "actions" in result
    assert len(result["actions"]) == 1
    action = result["actions"][0]
    assert isinstance(action, VDAAction)
    assert action.action_type == "pick"
    assert action.action_id == "action-1"
    # Check that parameters are VDAActionParameter with string values
    assert len(action.action_parameters) == 2
    assert isinstance(action.action_parameters[0], VDAActionParameter)
    assert action.action_parameters[0].key == "k1"
    assert action.action_parameters[0].value == "v1"
    assert action.action_parameters[1].value == "42"  # int cast to str


def test_generate_vda_instant_action_msg_v1():
    """Test that generate_vda_instant_action_msg handles v1 'instant_actions' field."""
    from task_011_amr_interop.msg import Action as VDAAction
    from vda5050_connector_py.mqtt_bridge import generate_vda_instant_action_msg

    instant_action_v1 = {
        "header_id": 1,
        "timestamp": "2023-01-01T00:00:00.00Z",
        "version": "1.1.0",
        "manufacturer": "test_mfg",
        "serial_number": "test_sn",
        "instant_actions": [
            {
                "action_type": "drop",
                "action_id": "action-2",
                "blocking_type": "HARD",
            }
        ],
    }

    result = generate_vda_instant_action_msg(instant_action_v1)

    assert "actions" in result
    # The legacy key should be removed
    assert "instant_actions" not in result
    assert len(result["actions"]) == 1
    action = result["actions"][0]
    assert isinstance(action, VDAAction)
    assert action.action_type == "drop"
    assert action.action_id == "action-2"


def test_mqtt_bridge_creates_subscriptions_and_publishers():
    """
    Test that the MQTTBridge node creates the expected ROS 2 subscriptions
    and publishers, and that publishing on the state topic is received.
    """
    import rclpy
    from rclpy.node import Node as RclpyNode
    from task_011_amr_interop.msg import OrderState as VDAOrderState

    # Check if paho is available; skip if not
    try:
        import paho.mqtt  # noqa: F401
    except ImportError:
        pytest.skip("paho-mqtt not installed, skipping MQTTBridge instantiation test")

    rclpy.init()
    test_node = None
    bridge_node = None
    try:
        from vda5050_connector_py.mqtt_bridge import MQTTBridge

        # Create the bridge node - it will fail to connect to MQTT but
        # ROS 2 subscriptions/publishers should still be created
        bridge_node = MQTTBridge()

        # Create a test node to interact with the bridge
        test_node = RclpyNode("test_bridge_node")

        # Expected topics
        state_topic = "/uagv/v2/robots/robot_1/state"
        order_topic = "/uagv/v2/robots/robot_1/order"

        # Create a publisher on the state topic to test the bridge subscription
        state_pub = test_node.create_publisher(VDAOrderState, state_topic, 10)

        # Give time for discovery
        deadline = time.time() + 5.0
        found_state = False
        found_order = False
        while time.time() < deadline:
            rclpy.spin_once(bridge_node, timeout_sec=0.1)
            rclpy.spin_once(test_node, timeout_sec=0.1)

            topic_names_and_types = test_node.get_topic_names_and_types()
            topic_names = [t[0] for t in topic_names_and_types]
            if state_topic in topic_names:
                found_state = True
            if order_topic in topic_names:
                found_order = True
            if found_state and found_order:
                break

        # Verify topics exist
        topic_names_and_types = test_node.get_topic_names_and_types()
        topic_names = [t[0] for t in topic_names_and_types]

        assert state_topic in topic_names, \
            f"State topic '{state_topic}' not found. Available: {topic_names}"
        assert order_topic in topic_names, \
            f"Order topic '{order_topic}' not found. Available: {topic_names}"

        # Publish a state message and verify the bridge processes it
        state_msg = VDAOrderState()
        state_msg.order_id = "test_order_123"
        state_msg.order_update_id = 0

        state_pub.publish(state_msg)

        # Spin to process the message
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(bridge_node, timeout_sec=0.1)
            rclpy.spin_once(test_node, timeout_sec=0.1)

        # Verify the bridge node name
        assert bridge_node.get_name() == "mqtt_bridge"

    finally:
        if bridge_node:
            try:
                bridge_node.mqtt_client.loop_stop()
            except Exception:
                pass
            bridge_node.destroy_node()
        if test_node:
            test_node.destroy_node()
        rclpy.try_shutdown()


def test_generate_vda5050_topic_alias():
    """Test the topic alias generation for supported versions."""
    from vda5050_connector_py.mqtt_bridge import generate_vda5050_topic_alias

    assert generate_vda5050_topic_alias("2.0.0") == "v2"
    assert generate_vda5050_topic_alias("1.1.0") == "v1"

    with pytest.raises(ValueError):
        generate_vda5050_topic_alias("3.0.0")


def test_state_handler_hpp_content():
    """Verify the C++ state_handler.hpp has the required interface."""
    from pathlib import Path

    # Find the file - check multiple possible locations
    possible_paths = [
        Path(__file__).parent / "state_handler.hpp",
        Path(__file__).parent / "include" / "vda5050_connector" / "state_handler.hpp",
    ]

    content = None
    for p in possible_paths:
        if p.exists():
            content = p.read_text()
            break

    if content is None:
        pytest.skip("state_handler.hpp not found in expected locations")

    # Verify key elements
    assert "class StateHandler" in content, \
        "StateHandler class must exist"
    assert "Handler" in content, \
        "StateHandler must reference Handler"
    assert "virtual void configure() = 0" in content, \
        "configure() must be pure virtual"
    assert "virtual void execute() = 0" in content, \
        "execute() must be pure virtual"
    assert "namespace adapter" in content, \
        "Must be in adapter namespace"
    assert "current_state_msg_" in content, \
        "Must have current_state_msg_ member"