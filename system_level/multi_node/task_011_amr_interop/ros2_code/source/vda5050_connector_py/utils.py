# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.

from datetime import datetime
import re
import json
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rosidl_runtime_py import message_to_ordereddict


def get_vda5050_ts():
    """
    Generate timestamp string using VDA5050 required format.

    The timestamp is in format ISO 8601 (UTC)
    YYYY-MM-DDTHH:mm:ss.ssZ (e.g."2017-04-15T11:40:03.12Z")

    Returns
    -------
        str: ISO 8601 UTC timestamp YYYY-MM-DDTHH:mm:ss.ssZ

    """
    d = datetime.utcnow()
    ts = d.isoformat()
    ts = ts[:-3]
    return f"{ts}Z"


def read_bool_parameter(node: Node, param_name: str, alternative: bool) -> bool:
    """Declare and read a bool parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == bool else param.get_parameter_value().bool_value


def read_str_parameter(node: Node, param_name: str, alternative: str) -> str:
    """Declare and read a string parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == str else param.get_parameter_value().string_value


def read_int_parameter(node: Node, param_name: str, alternative: int) -> int:
    """Declare and read a int parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == int else param.get_parameter_value().integer_value


def read_double_parameter(node: Node, param_name: str, alternative: float) -> float:
    """Declare and read a double (float) parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == float else param.get_parameter_value().double_value


def read_str_array_parameter(node: Node, param_name: str, alternative: list) -> list:
    """Declare and read a string array parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == list else param.get_parameter_value().string_array_value


def json_camel_to_snake_case(s):
    """
    Convert camel case JSON message to snake case.

    Converts all JSON message keys recursively from camel case to snake
    case e.g. camelCase to camel_case. Used to transform VDA5050 messages
    from MQTT topics to ROS2 vda5050_msgs.

    Args:
    ----
        s (str|bytes): JSON message as string or bytes

    """
    def snake_case_dict(obj):
        """Replace dict camelCase keys by its snake case equivalent."""
        for key in obj.copy():
            new_key = re.sub(r"(?<!^)(?=[A-Z])", "_", key).lower()
            if new_key != key:
                obj[new_key] = obj[key]
                del obj[key]
        return obj

    return json.loads(s, object_hook=snake_case_dict)


def json_snake_to_camel_case(s):
    """
    Convert snake case JSON message to camel case.

    Converts all JSON message keys recursively from snake case to camel
    case e.g. snake_case to snakeCase. Used to transform ROS2 vda5050_msgs
    messages to VDA5050 MQTT messages.

    Args:
    ----
        s (str|bytes): JSON message as string or bytes

    """
    def to_camel_case(snake_str):
        """Convert snake case string to camel case."""
        components = snake_str.split("_")
        return components[0] + "".join(x.title() for x in components[1:])

    def camel_case_dict(obj):
        """Replace dict snake_case keys by its camelCase equivalent."""
        for key in obj.copy():
            new_key = to_camel_case(key)
            if new_key != key:
                obj[new_key] = obj[key]
                del obj[key]
        return obj

    return json.loads(s, object_hook=camel_case_dict)


def convert_ros_message_to_json(msg):
    """
    Convert a ROS2 message to a JSON string with camelCase keys.

    Args:
    ----
        msg: ROS2 message object

    Returns
    -------
        str: JSON string with camelCase keys

    """
    ordered_dict = message_to_ordereddict(msg)
    json_str = json.dumps(ordered_dict)
    result = json_snake_to_camel_case(json_str)
    return json.dumps(result)


VALID_MAJOR_VERSIONS = ["v1", "v2"]


def get_vda5050_mqtt_topic(
    manufacturer,
    serial_number,
    topic,
    interface_name="uagv",
    major_version="v1",
):
    """
    Get a VDA5050 MQTT topic string.

    Args:
    ----
        manufacturer (str): Manufacturer name.
        serial_number (str): Serial number.
        topic (str): Topic name.
        interface_name (str): Interface name (default: 'uagv').
        major_version (str): Major version alias (default: 'v1').

    Returns
    -------
        str: MQTT topic string.

    Raises
    ------
        ValueError: If major_version is not valid.

    """
    if major_version not in VALID_MAJOR_VERSIONS:
        raise ValueError(
            f"Invalid major version '{major_version}'. "
            f"Valid versions are: {VALID_MAJOR_VERSIONS}"
        )
    return f"{interface_name}/{major_version}/{manufacturer}/{serial_number}/{topic}"


def get_vda5050_ros2_topic(
    manufacturer,
    serial_number,
    topic,
    interface_name="uagv",
    major_version="v1",
):
    """
    Get a VDA5050 ROS2 topic string.

    Args:
    ----
        manufacturer (str): Manufacturer name.
        serial_number (str): Serial number.
        topic (str): Topic name.
        interface_name (str): Interface name (default: 'uagv').
        major_version (str): Major version alias (default: 'v1').

    Returns
    -------
        str: ROS2 topic string (prefixed with /).

    Raises
    ------
        ValueError: If major_version is not valid.

    """
    if major_version not in VALID_MAJOR_VERSIONS:
        raise ValueError(
            f"Invalid major version '{major_version}'. "
            f"Valid versions are: {VALID_MAJOR_VERSIONS}"
        )
    return f"/{interface_name}/{major_version}/{manufacturer}/{serial_number}/{topic}"