Here is the converted ROS2 code:
```python
#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import rclpy
from rclpy.node import Node
import os
import time

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rclpy.init()
    node = Node('spawn_sdf_model_client')
    node.get_logger().info("Waiting for service %s/spawn_sdf_model"%gazebo_namespace)
    client = node.create_client(SpawnModel, gazebo_namespace+'/spawn_sdf_model')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    try:
      request = SpawnModel.Request()
      request.model_name = model_name
      request.model_xml = model_xml
      request.robot_namespace = robot_namespace
      request.initial_pose = initial_pose
      request.reference_frame = reference_frame
      future = client.call_async(request)
      rclpy.spin_until_future_complete(node, future)
      response = future.result()
      node.get_logger().info("Spawn status: %s"%response.status_message)
      return response.success
    except Exception as e:
      node.get_logger().error("Service call failed: %s" % e)
    finally:
      node.destroy_node()
      rclpy.shutdown()


def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rclpy.init()
    node = Node('spawn_urdf_model_client')
    node.get_logger().info("Waiting for service %s/spawn_urdf_model"%gazebo_namespace)
    client = node.create_client(SpawnModel, gazebo_namespace+'/spawn_urdf_model')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    try:
      request = SpawnModel.Request()
      request.model_name = model_name
      request.model_xml = model_xml
      request.robot_namespace = robot_namespace
      request.initial_pose = initial_pose
      request.reference_frame = reference_frame
      future = client.call_async(request)
      rclpy.spin_until_future_complete(node, future)
      response = future.result()
      node.get_logger().info("Spawn status: %s"%response.status_message)
      return response.success
    except Exception as e:
      node.get_logger().error("Service call failed: %s" % e)
    finally:
      node.destroy_node()
      rclpy.shutdown()


def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    rclpy.init()
    node = Node('set_model_configuration_client')
    node.get_logger().info("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    client = node.create_client(SetModelConfiguration, gazebo_namespace+'/set_model_configuration')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    try:
      request = SetModelConfiguration.Request()
      request.model_name = model_name
      request.model_param_name = model_param_name
      request.joint_names = joint_names
      request.joint_positions = joint_positions
      future = client.call_async(request)
      rclpy.spin_until_future_complete(node, future)
      response = future.result()
      node.get_logger().info("Set model configuration status: %s"%response.success)
      return response.success
    except Exception as e:
      node.get_logger().error("Service call failed: %s" % e)
    finally:
      node.destroy_node()
      rclpy.shutdown()