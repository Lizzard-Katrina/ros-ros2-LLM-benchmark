#!/usr/bin/env python3
# Wrappers around the services provided by rosified gazebo

import sys
import os
import time

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


_node = None


def _get_node():
    global _node
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = Node('gazebo_service_clients')
    return _node


def _wait_for_client(node, client, service_name):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for service {service_name}')


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = _get_node()
    service_name = f'{gazebo_namespace}/spawn_sdf_model'
    node.get_logger().info(f'Waiting for service {service_name}')
    client = node.create_client(SpawnModel, service_name)
    _wait_for_client(node, client, service_name)

    try:
        req = SpawnModel.Request()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame

        node.get_logger().info(f'Calling service {service_name}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is None:
            node.get_logger().error('Service call failed: no response')
            return False

        resp = future.result()
        node.get_logger().info(f'Spawn status: {resp.status_message}')
        return resp.success
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
        return False


def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = _get_node()
    service_name = f'{gazebo_namespace}/spawn_urdf_model'
    node.get_logger().info(f'Waiting for service {service_name}')
    client = node.create_client(SpawnModel, service_name)
    _wait_for_client(node, client, service_name)
    try:
        req = SpawnModel.Request()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame

        node.get_logger().info(f'Calling service {service_name}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is None:
            node.get_logger().error('Service call failed: no response')
            return False

        resp = future.result()
        node.get_logger().info(f'Spawn status: {resp.status_message}')
        return resp.success
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
        return False


def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    node = _get_node()
    service_name = f'{gazebo_namespace}/set_model_configuration'
    node.get_logger().info(f'Waiting for service {service_name}')
    client = node.create_client(SetModelConfiguration, service_name)
    _wait_for_client(node, client, service_name)
    node.get_logger().info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
    time.sleep(1)
    try:
        req = SetModelConfiguration.Request()
        req.model_name = model_name
        if hasattr(req, 'urdf_param_name'):
            req.urdf_param_name = model_param_name
        elif hasattr(req, 'model_param_name'):
            req.model_param_name = model_param_name
        req.joint_names = joint_names
        req.joint_positions = joint_positions

        node.get_logger().info(f'Calling service {service_name}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is None:
            node.get_logger().error('Service call failed: no response')
            return False

        resp = future.result()
        status_message = getattr(resp, 'status_message', '')
        if status_message:
            node.get_logger().info(f'Set model configuration status: {status_message}')
        node.get_logger().info(f'Set model configuration success: {resp.success}')
        return resp.success
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
        return False