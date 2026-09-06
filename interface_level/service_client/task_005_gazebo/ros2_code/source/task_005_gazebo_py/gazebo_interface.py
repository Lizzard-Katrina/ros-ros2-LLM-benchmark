#! /usr/bin/env python3
# Wrappers around the services provided by rosified gazebo

import sys
import rclpy
import os
import time

from task_005_gazebo.srv import SpawnEntity, SetModelConfiguration
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = rclpy.create_node('spawn_sdf_model_client_node')
    service_name = gazebo_namespace + '/spawn_sdf_model'
    node.get_logger().info("Waiting for service %s" % service_name)
    cli = node.create_client(SpawnEntity, service_name)
    cli.wait_for_service()
    try:
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace
        request.initial_pose = initial_pose
        request.reference_frame = reference_frame
        node.get_logger().info("Calling service %s" % service_name)
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info("Spawn status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False


def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = rclpy.create_node('spawn_urdf_model_client_node')
    service_name = gazebo_namespace + '/spawn_urdf_model'
    node.get_logger().info("Waiting for service %s" % service_name)
    cli = node.create_client(SpawnEntity, service_name)
    cli.wait_for_service()
    try:
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace
        request.initial_pose = initial_pose
        request.reference_frame = reference_frame
        node.get_logger().info("Calling service %s" % service_name)
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info("Spawn status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False


def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    node = rclpy.create_node('set_model_configuration_client_node')
    service_name = gazebo_namespace + '/set_model_configuration'
    node.get_logger().info("Waiting for service %s" % service_name)
    cli = node.create_client(SetModelConfiguration, service_name)
    cli.wait_for_service()
    node.get_logger().info("temporary hack to fix the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
    time.sleep(1)
    try:
        request = SetModelConfiguration.Request()
        request.model_name = model_name
        request.urdf_param_name = model_param_name
        request.joint_names = joint_names
        request.joint_positions = joint_positions
        node.get_logger().info("Calling service %s" % service_name)
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info("Set model configuration status: %s" % response.status_message)
        node.destroy_node()
        return response.success
    except Exception as e:
        print("Service call failed: %s" % e)
        node.destroy_node()
        return False