#!/usr/bin/env python3
# This file exists at the package root for oracle test compatibility.
# The actual implementation is in vda5050_connector_py/mqtt_bridge.py

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.

# Python dependencies
import copy
import json
import ssl
import os

# ROS dependencies / utils
from rclpy.node import Node

from vda5050_connector_py.utils import get_vda5050_mqtt_topic
from vda5050_connector_py.utils import get_vda5050_ros2_topic
from vda5050_connector_py.utils import json_camel_to_snake_case
from vda5050_connector_py.utils import read_str_parameter, read_int_parameter
from vda5050_connector_py.utils import convert_ros_message_to_json
from vda5050_connector_py.utils import get_vda5050_ts

from vda5050_connector_py.vda5050_controller import DEFAULT_PROTOCOL_VERSION, SUPPORTED_PROTOCOL_VERSIONS

# ROS msgs / srvs / actions
from task_011_amr_interop.msg import Action as VDAAction
from task_011_amr_interop.msg import ActionParameter as VDAActionParameter
from task_011_amr_interop.msg import Connection as VDAConnection
from task_011_amr_interop.msg import ControlPoint as VDAControlPoint
from task_011_amr_interop.msg import Edge as VDAEdge
from task_011_amr_interop.msg import InstantActions as VDAInstantActions
from task_011_amr_interop.msg import Node as VDANode
from task_011_amr_interop.msg import NodePosition as VDANodePosition
from task_011_amr_interop.msg import Order as VDAOrder
from task_011_amr_interop.msg import OrderState as VDAOrderState
from task_011_amr_interop.msg import Trajectory as VDATrajectory
from task_011_amr_interop.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """Convert an Order message into a ROS2 Order message represented as a dict."""
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        for k in ["x", "y", "theta"]:
            node["node_position"][k] = float(node["node_position"][k])
        node["node_position"] = VDANodePosition(**node["node_position"])
        for action in node["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        node["actions"] = [VDAAction(**action) for action in node["actions"]]

    vda_order["nodes"] = [VDANode(**node) for node in vda_order["nodes"]]
    for edge in vda_order["edges"]:
        for action in edge["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        edge["actions"] = [VDAAction(**action) for action in edge["actions"]]

        for k in [
            "max_speed", "max_height", "min_height",
            "orientation", "max_rotation_speed", "length",
        ]:
            if k in edge:
                edge[k] = float(edge[k])

        if "trajectory" in edge:
            edge["trajectory"] = VDATrajectory(
                degree=float(edge["trajectory"]["degree"]),
                knot_vector=edge["trajectory"]["knot_vector"],
                control_points=[
                    VDAControlPoint(
                        x=float(cp["x"]),
                        y=float(cp["y"]),
                        orientation=float(cp["orientation"]),
                        weight=float(cp.get("weight", 1)),
                    )
                    for cp in edge["trajectory"]["control_points"]
                ],
            )
    vda_order["edges"] = [VDAEdge(**edge) for edge in vda_order["edges"]]
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    """Convert an InstantAction message into a ROS2 InstantActions message dict."""
    vda_instant_action = copy.deepcopy(instant_action)

    if "instant_actions" in vda_instant_action:
        actions_list = vda_instant_action["instant_actions"]
    else:
        actions_list = vda_instant_action["actions"]

    for action in actions_list:
        if "action_parameters" in action:
            action["action_parameters"] = [
                VDAActionParameter(
                    key=action_parameter["key"],
                    value=str(action_parameter["value"]),
                )
                for action_parameter in action["action_parameters"]
            ]

    vda_instant_action["actions"] = [VDAAction(**action) for action in actions_list]

    if "instant_actions" in vda_instant_action:
        del vda_instant_action["instant_actions"]

    return vda_instant_action


def generate_vda5050_topic_alias(vda_version):
    """Create an alias for the current vda5050 version."""
    if vda_version in SUPPORTED_PROTOCOL_VERSIONS:
        return f"v{vda_version[0]}"
    else:
        raise ValueError(
            f"Invalid protocol major version. Supported versions are: {SUPPORTED_PROTOCOL_VERSIONS},"
            f"but got {vda_version}"
        )


def _get_mqtt_client_module():
    """Lazily import paho.mqtt.client to avoid import errors when not installed."""
    from paho.mqtt import client as mqtt_client
    from paho.mqtt.client import error_string
    return mqtt_client, error_string


class MQTTBridge(Node):
    """Translates VDA5050 MQTT messages from and to ROS2."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        mqtt_client_mod, self._error_string = _get_mqtt_client_module()

        mqtt_address = read_str_parameter(self, "mqtt_address", "localhost")
        mqtt_port = read_int_parameter(self, "mqtt_port", 1883)
        mqtt_username = read_str_parameter(self, "mqtt_username", "")
        mqtt_password = read_str_parameter(self, "mqtt_password", "")

        self.vda5050_version = read_str_parameter(self, "vda5050_protocol_version", "2.0.0")
        self.vda5050_version_alias = generate_vda5050_topic_alias(self.vda5050_version)

        self._manufacturer_name = read_str_parameter(self, "manufacturer_name", "robots")
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")
        self._interface_name = read_str_parameter(self, "interface_name", "uagv")

        self.mqtt_client = mqtt_client_mod.Client()
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.on_message = self.on_message_mqtt
        self.mqtt_client.on_disconnect = self.on_disconnect_mqtt

        if mqtt_username:
            self.mqtt_client.tls_set(
                ca_certs=os.getenv(
                    key="VDA5050_CONNECTOR_TLS_CA_CERT",
                    default="/etc/ssl/certs/ca-certificates.crt",
                ),
                tls_version=ssl.PROTOCOL_TLSv1_2,
            )
            self.mqtt_client.username_pw_set(username=mqtt_username, password=mqtt_password)

        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )

        will_payload = convert_ros_message_to_json(
            VDAConnection(
                header_id=0,
                version=self.vda5050_version,
                timestamp="1970-01-01T12:00:00.00Z",
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                connection_state=VDAConnection.CONNECTIONBROKEN,
            )
        )
        self.mqtt_client.will_set(topic=will_topic, payload=will_payload, qos=1, retain=True)

        self._last_connection_msg = None
        self._mqtt_address = mqtt_address
        self._mqtt_port = int(mqtt_port)
        self._connect_to_broker()
        self.mqtt_client.loop_start()

        self._connect_timer = self.create_timer(timer_period_sec=5.0, callback=self._connect_to_broker)
        self.on_configure()
        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    def _connect_to_broker(self):
        if not self.mqtt_client.is_connected():
            try:
                self.mqtt_client.connect_async(host=self._mqtt_address, port=self._mqtt_port)
            except Exception:
                pass

    def on_connect_mqtt(self, client, userdata, flags, rc):
        if rc == 0:
            if hasattr(self, '_connect_timer'):
                self._connect_timer.cancel()
            self.mqtt_client.subscribe(get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name, serial_number=self._serial_number,
                topic="order", major_version=self.vda5050_version_alias,
                interface_name=self._interface_name))
            self.mqtt_client.subscribe(get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name, serial_number=self._serial_number,
                topic="instantActions", major_version=self.vda5050_version_alias,
                interface_name=self._interface_name))
            self._publish_connection(msg=VDAConnection(
                header_id=0, version=self.vda5050_version, timestamp=get_vda5050_ts(),
                manufacturer=self._manufacturer_name, serial_number=self._serial_number,
                connection_state=VDAConnection.ONLINE))

    def on_message_mqtt(self, client, userdata, msg):
        try:
            msg_json = json_camel_to_snake_case(msg.payload)
        except json.decoder.JSONDecodeError:
            return
        try:
            if msg.topic.endswith("order"):
                vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                self._order_pub.publish(msg=vda_order_msg)
            if msg.topic.endswith("instantActions"):
                vda_instant_actions_message = VDAInstantActions(**generate_vda_instant_action_msg(msg_json))
                self._instant_actions_pub.publish(msg=vda_instant_actions_message)
        except KeyError:
            return

    def on_disconnect_mqtt(self, client, userdata, rc):
        pass

    def on_configure(self):
        self._order_pub = self.create_publisher(msg_type=VDAOrder,
            topic=get_vda5050_ros2_topic(manufacturer=self._manufacturer_name,
                serial_number=self._serial_number, topic="order",
                major_version=self.vda5050_version_alias, interface_name=self._interface_name),
            qos_profile=10)
        self._instant_actions_pub = self.create_publisher(msg_type=VDAInstantActions,
            topic=get_vda5050_ros2_topic(manufacturer=self._manufacturer_name,
                serial_number=self._serial_number, topic="instantActions",
                major_version=self.vda5050_version_alias, interface_name=self._interface_name),
            qos_profile=10)
        self._state_sub = self.create_subscription(msg_type=VDAOrderState,
            topic=get_vda5050_ros2_topic(manufacturer=self._manufacturer_name,
                serial_number=self._serial_number, topic="state",
                major_version=self.vda5050_version_alias, interface_name=self._interface_name),
            callback=self._publish_state, qos_profile=10)
        self._connection_sub = self.create_subscription(msg_type=VDAConnection,
            topic=get_vda5050_ros2_topic(manufacturer=self._manufacturer_name,
                serial_number=self._serial_number, topic="connection",
                major_version=self.vda5050_version_alias, interface_name=self._interface_name),
            callback=self._publish_connection, qos_profile=10)
        self._visualization_sub = self.create_subscription(msg_type=VDAVisualization,
            topic=get_vda5050_ros2_topic(manufacturer=self._manufacturer_name,
                serial_number=self._serial_number, topic="visualization",
                major_version=self.vda5050_version_alias, interface_name=self._interface_name),
            callback=self._publish_visualization, qos_profile=10)

    def on_shutdown(self):
        offline_message = VDAConnection(
            header_id=0, version=self.vda5050_version, timestamp=get_vda5050_ts(),
            manufacturer=self._manufacturer_name, serial_number=self._serial_number,
            connection_state=VDAConnection.OFFLINE)
        if self._last_connection_msg:
            offline_message.header_id = self._last_connection_msg.header_id + 1
        self._publish_connection(msg=offline_message)
        self.mqtt_client.disconnect()

    def _publish_to_topic(self, msg, topic):
        json_msg = convert_ros_message_to_json(msg)
        self.mqtt_client.publish(topic, json_msg)

    def _publish_state(self, msg):
        topic = get_vda5050_mqtt_topic(manufacturer=self._manufacturer_name,
            serial_number=self._serial_number, topic="state",
            major_version=self.vda5050_version_alias, interface_name=self._interface_name)
        self._publish_to_topic(msg, topic)

    def _publish_connection(self, msg):
        self._last_connection_msg = msg
        topic = get_vda5050_mqtt_topic(manufacturer=self._manufacturer_name,
            serial_number=self._serial_number, topic="connection",
            major_version=self.vda5050_version_alias, interface_name=self._interface_name)
        self._publish_to_topic(msg, topic)

    def _publish_visualization(self, msg):
        topic = get_vda5050_mqtt_topic(manufacturer=self._manufacturer_name,
            serial_number=self._serial_number, topic="visualization",
            major_version=self.vda5050_version_alias, interface_name=self._interface_name)
        self._publish_to_topic(msg, topic)