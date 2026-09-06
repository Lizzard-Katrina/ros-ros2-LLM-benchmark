#!/usr/bin/env python3
"""Runtime test for task_011_amr_interop."""

import copy
import json
import os
import re
import time
import pytest
from pathlib import Path

import rclpy
from rclpy.node import Node


def test_generate_vda_instant_action_msg_v2():
    """Test that generate_vda_instant_action_msg handles v2 'actions' field."""
    from vda5050_connector_py.mqtt_bridge import generate_vda_instant_action_msg
    from task_011_amr_interop.msg import InstantActions as VDAInstantActions

    instant_action_v2 = {
        "header_id": 1,
        "timestamp": "2023-01-01T00:00:00.00Z",
        "version": "2.0.0",
        "manufacturer": "test_mfg",
        "serial_number": "test_sn",
        "actions": [
            {
                "action_type": "startPause",
                "action_id": "action-001",
                "blocking_type": "NONE",
            }
        ],
    }

    result = generate_vda_instant_action_msg(instant_action_v2)
    msg = VDAInstantActions(**result)
    assert len(msg.actions) == 1
    assert msg.actions[0].action_type == "startPause"
    assert msg.actions[0].action_id == "action-001"


def test_generate_vda_instant_action_msg_v1():
    """Test that generate_vda_instant_action_msg handles v1 'instant_actions' field."""
    from vda5050_connector_py.mqtt_bridge import generate_vda_instant_action_msg
    from task_011_amr_interop.msg import InstantActions as VDAInstantActions

    instant_action_v1 = {
        "header_id": 2,
        "timestamp": "2023-01-01T00:00:00.00Z",
        "version": "1.1.0",
        "manufacturer": "test_mfg",
        "serial_number": "test_sn",
        "instant_actions": [
            {
                "action_type": "stopPause",
                "action_id": "action-002",
                "blocking_type": "HARD",
                "action_parameters": [
                    {"key": "param1", "value": 42},
                    {"key": "param2", "value": True},
                ],
            }
        ],
    }

    result = generate_vda_instant_action_msg(instant_action_v1)
    msg = VDAInstantActions(**result)
    assert len(msg.actions) == 1
    assert msg.actions[0].action_type == "stopPause"
    assert msg.actions[0].action_id == "action-002"
    # Verify parameters are cast to string
    assert msg.actions[0].action_parameters[0].value == "42"
    assert msg.actions[0].action_parameters[1].value == "True"


def test_generate_vda_order_msg():
    """Test that generate_vda_order_msg produces a valid Order dict."""
    from vda5050_connector_py.mqtt_bridge import generate_vda_order_msg
    from task_011_amr_interop.msg import Order as VDAOrder

    order = {
        "header_id": 0,
        "timestamp": "2023-01-01T00:00:00.00Z",
        "version": "2.0.0",
        "manufacturer": "test_mfg",
        "serial_number": "test_sn",
        "order_id": "order-001",
        "order_update_id": 0,
        "nodes": [
            {
                "node_id": "node1",
                "sequence_id": 0,
                "released": True,
                "node_position": {
                    "x": 1.0,
                    "y": 2.0,
                    "theta": 0.0,
                    "map_id": "map",
                },
                "actions": [],
            }
        ],
        "edges": [],
    }

    result = generate_vda_order_msg(order)
    msg = VDAOrder(**result)
    assert msg.order_id == "order-001"
    assert len(msg.nodes) == 1
    assert msg.nodes[0].node_id == "node1"
    assert msg.nodes[0].node_position.x == 1.0


def test_utils_topic_generation():
    """Test the utility functions for topic generation."""
    from vda5050_connector_py.utils import get_vda5050_mqtt_topic, get_vda5050_ros2_topic

    mqtt_topic = get_vda5050_mqtt_topic(
        manufacturer="robots",
        serial_number="robot_1",
        topic="state",
        major_version="v2",
        interface_name="uagv",
    )
    assert mqtt_topic == "uagv/v2/robots/robot_1/state"

    ros2_topic = get_vda5050_ros2_topic(
        manufacturer="robots",
        serial_number="robot_1",
        topic="state",
        major_version="v2",
        interface_name="uagv",
    )
    assert ros2_topic == "/uagv/v2/robots/robot_1/state"

    # Test invalid version raises
    with pytest.raises(ValueError):
        get_vda5050_mqtt_topic(
            manufacturer="M",
            serial_number="SN",
            topic="order",
            major_version="v123",
            interface_name="uagv",
        )