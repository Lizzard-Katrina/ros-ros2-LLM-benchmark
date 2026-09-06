#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import rclpy
from rclpy.node import Node
import os
import time

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

# Conditionally import gazebo_msgs; if not available, define stubs
# so the module can still be loaded and tested with alternative services.
try:
    from gazebo_msgs.srv import SpawnEntity, SetModelConfiguration
    _HAS_GAZEBO_MSGS = True
except ImportError:
    _HAS_GAZEBO_MSGS = False


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    if not _HAS_GAZEBO_MSGS:
        raise ImportError("gazebo_msgs is not available")
    node = rclpy.create_node('spawn_sdf_model_client_node')
    node.get_logger().info("Waiting for service %s/spawn_sdf_model" % gazebo_namespace)
    client = node.create_client(SpawnEntity, gazebo_namespace + '/spawn_sdf_model')
    client.wait_for_service()
    try:
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace
        request.initial_pose = initial_pose
        request.reference_frame = reference_frame
        node.get_logger().info("Calling service %s/spawn_sdf_model" % gazebo_namespace)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            node.get_logger().error("Service call failed (no response)")
            node.destroy_node()
            return False
        node.get_logger().info("Spawn status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False


def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    if not _HAS_GAZEBO_MSGS:
        raise ImportError("gazebo_msgs is not available")
    node = rclpy.create_node('spawn_urdf_model_client_node')
    node.get_logger().info("Waiting for service %s/spawn_urdf_model" % gazebo_namespace)
    client = node.create_client(SpawnEntity, gazebo_namespace + '/spawn_urdf_model')
    client.wait_for_service()
    try:
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace
        request.initial_pose = initial_pose
        request.reference_frame = reference_frame
        node.get_logger().info("Calling service %s/spawn_urdf_model" % gazebo_namespace)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            node.get_logger().error("Service call failed (no response)")
            node.destroy_node()
            return False
        node.get_logger().info("Spawn status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False


def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    if not _HAS_GAZEBO_MSGS:
        raise ImportError("gazebo_msgs is not available")
    node = rclpy.create_node('set_model_configuration_client_node')
    node.get_logger().info("Waiting for service %s/set_model_configuration" % gazebo_namespace)
    client = node.create_client(SetModelConfiguration, gazebo_namespace + '/set_model_configuration')
    client.wait_for_service()
    node.get_logger().info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
    time.sleep(1)
    try:
        request = SetModelConfiguration.Request()
        request.model_name = model_name
        request.urdf_param_name = model_param_name
        request.joint_names = joint_names
        request.joint_positions = joint_positions
        node.get_logger().info("Calling service %s/set_model_configuration" % gazebo_namespace)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            node.get_logger().error("Service call failed (no response)")
            node.destroy_node()
            return False
        node.get_logger().info("Set model configuration status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False