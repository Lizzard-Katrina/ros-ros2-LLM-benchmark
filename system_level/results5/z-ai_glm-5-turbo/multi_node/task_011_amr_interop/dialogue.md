# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: mqtt_bridge.py
----------------------------
#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Python dependencies
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
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
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import ActionParameter as VDAActionParameter
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import ControlPoint as VDAControlPoint
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition as VDANodePosition
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import Trajectory as VDATrajectory
from vda5050_msgs.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """
    Convert an Order message into a ROS2 Order message represented as a dict.

    Args:
    ----
        order (VDAOrder): VDA5050 Order message

    Returns
    -------
        Order dict message for building a ROS2 Order object

    """
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        # Force all numbers to float. Values with no decimals are
        # interpret as integers, causing the validation errors
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

        # Force all numbers to float. Values with no decimals are
        # interpreted as integers, causing the validation errors.
        for k in [
            "max_speed",
            "max_height",
            "min_height",
            "orientation",
            "max_rotation_speed",
            "length",
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
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    """
    # TODO [Task_011_B]: Implement VDA5050 version-agnostic instantAction mapping.
    # - The input 'instant_action' is a dict from MQTT.
    # - HACK: Handle the protocol evolution: v1 uses 'instant_actions' field, while v2 uses 'actions'.
    # - Ensure all 'action_parameters' values are cast to strings to satisfy VDAActionParameter requirements.
    # - Return a dict compatible with VDAInstantActions ROS 2 message construction.
    # END OF TODO
    """

def generate_vda5050_topic_alias(vda_version):
    """
    Create an alias for the current vda5050 version. The aliases are needed to
    create the mqtt topics.

    Args:
    ----
        vda_version (string): VDA5050 version with format x.x.x.

    Raises:
    ------
        ValueError if the alias is not within the supported values.

    Returns
    -------
        The alias of the version. For example, for the version '2.0.0', the alias is
        'v2'
    """
    if vda_version in SUPPORTED_PROTOCOL_VERSIONS:
        return f"v{vda_version[0]}"
    else:
        raise ValueError(
            f"Invalid protocol major version. Supported versions are: {SUPPORTED_PROTOCOL_VERSIONS},"
            f"but got {vda_version}"
        )

class MQTTBridge(Node):
    """Translates VDA5050 MQTT messages from and to ROS2."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Use default values if no parameters
        # are defined on launchfile. Provide the parameter when running the launchfile
        # by using ``foo.launch.py mqtt_address:=localhost mqtt_port:=1883 ...``
        mqtt_address = read_str_parameter(self, "mqtt_address", "localhost")
        mqtt_port = read_int_parameter(self, "mqtt_port", 1883)
        mqtt_username = read_str_parameter(self, "mqtt_username", "")
        mqtt_password = read_str_parameter(self, "mqtt_password", "")

        self.vda5050_version = read_str_parameter(self, "vda5050_protocol_version", "2.0.0")
        self.vda5050_version_alias = generate_vda5050_topic_alias(self.vda5050_version)

        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        self._interface_name = read_str_parameter(self, "interface_name", "uagv")

        # Configure MQTT
        self.mqtt_client = mqtt_client.Client()
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.on_message = self.on_message_mqtt
        self.mqtt_client.on_disconnect = self.on_disconnect_mqtt

        # Enable TLS if username is provided
        if mqtt_username:
            self.mqtt_client.tls_set(
                ca_certs=os.getenv(
                    key="VDA5050_CONNECTOR_TLS_CA_CERT",
                    default="/etc/ssl/certs/ca-certificates.crt",
                ),
                tls_version=ssl.PROTOCOL_TLSv1_2,
            )
            self.mqtt_client.username_pw_set(
                username=mqtt_username, password=mqtt_password
            )

        # Configure will message or last testament message
        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )

        # NOTE: will payload cannot be set dynamically or updated
        # without reconnecting, so some values are fixed. For the
        # timestamp, instead of using the time the connector started
        # the ts 0 is used which is easy to identify.
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
        self.mqtt_client.will_set(
            topic=will_topic, payload=will_payload, qos=1, retain=True
        )

        # Keep a copy of the last VDA5050 Connection message
        self._last_connection_msg = None

        # Connect to MQTT broker
        self._mqtt_address = mqtt_address
        self._mqtt_port = int(mqtt_port)
        self._connect_to_broker()

        self.mqtt_client.loop_start()

        self._connect_timer = self.create_timer(
            timer_period_sec=5.0,
            callback=self._connect_to_broker
        )

        self.on_configure()

        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    def _connect_to_broker(self):
        """Attempts to connect to the MQTT broker."""
        if not self.mqtt_client.is_connected():
            try:
                self.mqtt_client.connect_async(host=self._mqtt_address, port=self._mqtt_port)
                self.logger.info(f"Attempting to connect to MQTT broker at {self._mqtt_address}:{self._mqtt_port}...")
            except Exception as e:
                self.logger.error(f"Error during connection attempt: {e}. Will retry again.")
                pass

    def on_connect_mqtt(self, client, userdata, flags, rc):
        """MQTT client connect callback."""
        if rc == 0:
            self.logger.info("Connected to MQTT Broker!")

            # Cancel the connection timer
            if hasattr(self, '_connect_timer'):
                self._connect_timer.cancel()

            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="order",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="instantActions",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self._publish_connection(
                msg=VDAConnection(
                    header_id=0,
                    version=self.vda5050_version,
                    timestamp=get_vda5050_ts(),
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    connection_state=VDAConnection.ONLINE,
                )
            )

        else:
            self.logger.error("Failed to connect, return code %d\n", rc)

    def on_message_mqtt(self, client, userdata, msg):
        """MQTT client message callback."""
        try:
            msg_json = json_camel_to_snake_case(msg.payload)
            self.logger.debug(f"Received '{msg_json}' from '{msg.topic}' topic")
        except json.decoder.JSONDecodeError:
            self.logger.error(f"Failed to decode message: '{msg.payload}'")
            return

        try:
            if msg.topic.endswith("order"):
                vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                self._order_pub.publish(msg=vda_order_msg)
            if msg.topic.endswith("instantActions"):
                vda_instant_actions_message = VDAInstantActions(
                    **generate_vda_instant_action_msg(msg_json)
                )
                self._instant_actions_pub.publish(msg=vda_instant_actions_message)
        except KeyError as ex:
            self.logger.warn(f"Ignoring invalid VDA5050 message: {ex}.")
            return

    def on_disconnect_mqtt(self, client, userdata, rc):
        """MQTT client disconnect callback."""
        if rc != 0:
            self.logger.info(
                f"MQTT client disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect."
            )
        else:
            self.logger.info("Disconnected from MQTT Broker!")

    def on_configure(self):
        """
        # TODO [Task_011_C]: Orchestrate ROS 2 subscriptions for VDA5050 interoperability.
        # - Create subscriptions for 'state', 'connection', and 'visualization' topics.
        # - Use 'get_vda5050_ros2_topic' helper for topic naming.
        # - Map callbacks to their respective publish methods (e.g., _publish_state, _publish_connection).
        # - This method is the central hub connecting ROS 2 world to the MQTT bridge.
        # END OF TODO
        """
    def on_shutdown(self):
        """Perform all necessary teardown steps."""
        self.logger.info("Publishing offline Connection message")

        offline_message = VDAConnection(
            header_id=0,
            version=self.vda5050_version,
            timestamp=get_vda5050_ts(),
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            connection_state=VDAConnection.OFFLINE,
        )

        # Use the latest Connection message `header_id` if available
        if self._last_connection_msg:
            offline_message.header_id = self._last_connection_msg.header_id + 1

        self._publish_connection(msg=offline_message)

        self.logger.info("Unsubscribing from MQTT topics")
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )

        self.mqtt_client.disconnect()

    def _publish_to_topic(self, msg, topic):
        """
        Publish a ROS2 message to an MQTT topic.

        Args:
        ----
            msg (Any): VDA5050 ROS2 message.
            topic (str): topic for publishing the VDA5050 MQTT message.

        """
        json_msg = convert_ros_message_to_json(msg)
        self.logger.debug(f"Publishing MQTT message to topic {topic}: {json_msg}")
        self.mqtt_client.publish(topic, json_msg)

    def _publish_state(self, msg: VDAOrderState):
        """
        Publish ROS2 OrderState message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAOrderState): VDA5050 ROS2 OrderState message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="state",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_connection(self, msg: VDAConnection):
        """
        Publish VDA5050 ROS2 Connection message to the corresponding VDA5050 MQTT topic.

        Also updates a local copy of the last published VDA5050 Connection message to keep track
        of the latest ``header_id`` field. This is used to publish an offline Connection message
        when tearing down the node.

        Args:
        ----
            msg (VDAConnection): VDA5050 ROS2 Connection message.

        """
        # Update the last connection message
        self._last_connection_msg = msg
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_visualization(self, msg: VDAVisualization):
        """
        Publish ROS2 Visualization message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAVisualization): VDA5050 ROS2 Visualization message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="visualization",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

----------------------------

FILE_PATH: state_handler.hpp
----------------------------
// BSD 3-Clause License
//
// Copyright (c) 2022 InOrbit, Inc.
// Copyright (c) 2022 Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the InOrbit, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * Pre-compiler statements
 */
#pragma once

/**
 * C++ Libraries / header
 */
#include "vda5050_connector/handler.hpp"

namespace adapter
{
/**
 * TODO [Task_011_A]: Define the StateHandler abstract interface.
 * - The class must inherit from 'Handler'.
 * - Define two pure virtual methods: 'configure()' and 'execute()'.
 * - Add a protected member variable to hold the state, which subclasses will use.
 * - Style Constraint: Use 'vda5050_msgs::msg::State' as the state message type.
 * - Style Constraint: The member variable MUST be named 'current_state_msg_'.
 * END OF TODO 
*/
};

}  // namespace adapter

----------------------------


# LLM Response

[FILENAME: mqtt_bridge.py]
#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Python dependencies
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
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
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import ActionParameter as VDAActionParameter
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import ControlPoint as VDAControlPoint
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition as VDANodePosition
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import Trajectory as VDATrajectory
from vda5050_msgs.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """
    Convert an Order message into a ROS2 Order message represented as a dict.

    Args:
    ----
        order (VDAOrder): VDA5050 Order message

    Returns
    -------
        Order dict message for building a ROS2 Order object

    """
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        # Force all numbers to float. Values with no decimals are
        # interpret as integers, causing the validation errors
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

        # Force all numbers to float. Values with no decimals are
        # interpreted as integers, causing the validation errors.
        for k in [
            "max_speed",
            "max_height",
            "min_height",
            "orientation",
            "max_rotation_speed",
            "length",
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
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    vda_instant_action = copy.deepcopy(instant_action)
    actions_key = "actions" if "actions" in vda_instant_action else "instant_actions"
    raw_actions = vda_instant_action.get(actions_key, [])
    parsed_actions = []
    for action in raw_actions:
        if "action_parameters" in action:
            action["action_parameters"] = [
                VDAActionParameter(
                    key=ap["key"],
                    value=str(ap["value"])
                )
                for ap in action["action_parameters"]
            ]
        parsed_actions.append(VDAAction(**action))
    vda_instant_action["actions"] = parsed_actions
    if "instant_actions" in vda_instant_action:
        del vda_instant_action["instant_actions"]
    return vda_instant_action


def generate_vda5050_topic_alias(vda_version):
    """
    Create an alias for the current vda5050 version. The aliases are needed to
    create the mqtt topics.

    Args:
    ----
        vda_version (string): VDA5050 version with format x.x.x.

    Raises:
    ------
        ValueError if the alias is not within the supported values.

    Returns
    -------
        The alias of the version. For example, for the version '2.0.0', the alias is
        'v2'
    """
    if vda_version in SUPPORTED_PROTOCOL_VERSIONS:
        return f"v{vda_version[0]}"
    else:
        raise ValueError(
            f"Invalid protocol major version. Supported versions are: {SUPPORTED_PROTOCOL_VERSIONS},"
            f"but got {vda_version}"
        )

class MQTTBridge(Node):
    """Translates VDA5050 MQTT messages from and to ROS2."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Use default values if no parameters
        # are defined on launchfile. Provide the parameter when running the launchfile
        # by using ``foo.launch.py mqtt_address:=localhost mqtt_port:=1883 ...``
        mqtt_address = read_str_parameter(self, "mqtt_address", "localhost")
        mqtt_port = read_int_parameter(self, "mqtt_port", 1883)
        mqtt_username = read_str_parameter(self, "mqtt_username", "")
        mqtt_password = read_str_parameter(self, "mqtt_password", "")

        self.vda5050_version = read_str_parameter(self, "vda5050_protocol_version", "2.0.0")
        self.vda5050_version_alias = generate_vda5050_topic_alias(self.vda5050_version)

        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        self._interface_name = read_str_parameter(self, "interface_name", "uagv")

        # Configure MQTT
        self.mqtt_client = mqtt_client.Client()
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.on_message = self.on_message_mqtt
        self.mqtt_client.on_disconnect = self.on_disconnect_mqtt

        # Enable TLS if username is provided
        if mqtt_username:
            self.mqtt_client.tls_set(
                ca_certs=os.getenv(
                    key="VDA5050_CONNECTOR_TLS_CA_CERT",
                    default="/etc/ssl/certs/ca-certificates.crt",
                ),
                tls_version=ssl.PROTOCOL_TLSv1_2,
            )
            self.mqtt_client.username_pw_set(
                username=mqtt_username, password=mqtt_password
            )

        # Configure will message or last testament message
        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )

        # NOTE: will payload cannot be set dynamically or updated
        # without reconnecting, so some values are fixed. For the
        # timestamp, instead of using the time the connector started
        # the ts 0 is used which is easy to identify.
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
        self.mqtt_client.will_set(
            topic=will_topic, payload=will_payload, qos=1, retain=True
        )

        # Keep a copy of the last VDA5050 Connection message
        self._last_connection_msg = None

        # Connect to MQTT broker
        self._mqtt_address = mqtt_address
        self._mqtt_port = int(mqtt_port)
        self._connect_to_broker()

        self.mqtt_client.loop_start()

        self._connect_timer = self.create_timer(
            timer_period_sec=5.0,
            callback=self._connect_to_broker
        )

        self.on_configure()

        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    def _connect_to_broker(self):
        """Attempts to connect to the MQTT broker."""
        if not self.mqtt_client.is_connected():
            try:
                self.mqtt_client.connect_async(host=self._mqtt_address, port=self._mqtt_port)
                self.logger.info(f"Attempting to connect to MQTT broker at {self._mqtt_address}:{self._mqtt_port}...")
            except Exception as e:
                self.logger.error(f"Error during connection attempt: {e}. Will retry again.")
                pass

    def on_connect_mqtt(self, client, userdata, flags, rc):
        """MQTT client connect callback."""
        if rc == 0:
            self.logger.info("Connected to MQTT Broker!")

            # Cancel the connection timer
            if hasattr(self, '_connect_timer'):
                self._connect_timer.cancel()

            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="order",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="instantActions",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self._publish_connection(
                msg=VDAConnection(
                    header_id=0,
                    version=self.vda5050_version,
                    timestamp=get_vda5050_ts(),
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    connection_state=VDAConnection.ONLINE,
                )
            )

        else:
            self.logger.error("Failed to connect, return code %d\n", rc)

    def on_message_mqtt(self, client, userdata, msg):
        """MQTT client message callback."""
        try:
            msg_json = json_camel_to_snake_case(msg.payload)
            self.logger.debug(f"Received '{msg_json}' from '{msg.topic}' topic")
        except json.decoder.JSONDecodeError:
            self.logger.error(f"Failed to decode message: '{msg.payload}'")
            return

        try:
            if msg.topic.endswith("order"):
                vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                self._order_pub.publish(msg=vda_order_msg)
            if msg.topic.endswith("instantActions"):
                vda_instant_actions_message = VDAInstantActions(
                    **generate_vda_instant_action_msg(msg_json)
                )
                self._instant_actions_pub.publish(msg=vda_instant_actions_message)
        except KeyError as ex:
            self.logger.warn(f"Ignoring invalid VDA5050 message: {ex}.")
            return

    def on_disconnect_mqtt(self, client, userdata, rc):
        """MQTT client disconnect callback."""
        if rc != 0:
            self.logger.info(
                f"MQTT client disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect."
            )
        else:
            self.logger.info("Disconnected from MQTT Broker!")

    def on_configure(self):
        self._order_pub = self.create_publisher(VDAOrder, get_vda5050_ros2_topic("order"), 10)
        self._instant_actions_pub = self.create_publisher(VDAInstantActions, get_vda5050_ros2_topic("instant_actions"), 10)

        self.create_subscription(
            VDAOrderState,
            get_vda5050_ros2_topic("state"),
            self._publish_state,
            10
        )
        self.create_subscription(
            VDAConnection,
            get_vda5050_ros2_topic("connection"),
            self._publish_connection,
            10
        )
        self.create_subscription(
            VDAVisualization,
            get_vda5050_ros2_topic("visualization"),
            self._publish_visualization,
            10
        )

    def on_shutdown(self):
        """Perform all necessary teardown steps."""
        self.logger.info("Publishing offline Connection message")

        offline_message = VDAConnection(
            header_id=0,
            version=self.vda5050_version,
            timestamp=get_vda5050_ts(),
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            connection_state=VDAConnection.OFFLINE,
        )

        # Use the latest Connection message `header_id` if available
        if self._last_connection_msg:
            offline_message.header_id = self._last_connection_msg.header_id + 1

        self._publish_connection(msg=offline_message)

        self.logger.info("Unsubscribing from MQTT topics")
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )

        self.mqtt_client.disconnect()

    def _publish_to_topic(self, msg, topic):
        """
        Publish a ROS2 message to an MQTT topic.

        Args:
        ----
            msg (Any): VDA5050 ROS2 message.
            topic (str): topic for publishing the VDA5050 MQTT message.

        """
        json_msg = convert_ros_message_to_json(msg)
        self.logger.debug(f"Publishing MQTT message to topic {topic}: {json_msg}")
        self.mqtt_client.publish(topic, json_msg)

    def _publish_state(self, msg: VDAOrderState):
        """
        Publish ROS2 OrderState message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAOrderState): VDA5050 ROS2 OrderState message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="state",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_connection(self, msg: VDAConnection):
        """
        Publish VDA5050 ROS2 Connection message to the corresponding VDA5050 MQTT topic.

        Also updates a local copy of the last published VDA5050 Connection message to keep track
        of the latest ``header_id`` field. This is used to publish an offline Connection message
        when tearing down the node.

        Args:
        ----
            msg (VDAConnection): VDA5050 ROS2 Connection message.

        """
        # Update the last connection message
        self._last_connection_msg = msg
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_visualization(self, msg: VDAVisualization):
        """
        Publish ROS2 Visualization message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAVisualization): VDA5050 ROS2 Visualization message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="visualization",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

[FILENAME: state_handler.hpp]
// BSD 3-Clause License
//
// Copyright (c) 2022 InOrbit, Inc.
// Copyright (c) 2022 Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the InOrbit, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * Pre-compiler statements
 */
#pragma once

/**
 * C++ Libraries / header
 */
#include "vda5050_connector/handler.hpp"
#include "vda5050_msgs/msg/state.hpp"

namespace adapter
{

class StateHandler : public Handler
{
public:
  virtual ~StateHandler() = default;

  virtual void configure() = 0;
  virtual void execute() = 0;

protected:
  vda5050_msgs::msg::State current_state_msg_;
};

}  // namespace adapter