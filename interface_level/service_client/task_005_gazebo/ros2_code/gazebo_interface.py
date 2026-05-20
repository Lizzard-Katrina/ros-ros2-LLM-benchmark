#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import os
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import *
from gazebo_msgs.srv import SpawnEntity
try:
    from gazebo_msgs.srv import SetModelConfiguration
except ImportError:
    pass # May not be available in all ROS2 gazebo_msgs versions
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('spawn_sdf_model_client_node')
    
    service_name = gazebo_namespace + '/spawn_entity'
    node.get_logger().info("Waiting for service %s" % service_name)
    client = node.create_client(SpawnEntity, service_name)
    client.wait_for_service()
    
    try:
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        
        node.get_logger().info("Calling service %s" % service_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()
        
        if resp is not None:
            node.get_logger().info("Spawn status: %s" % resp.status_message)
            return resp.success
        else:
            node.get_logger().error("Service call failed")
            return False
    except Exception as e:
        node.get_logger().error("Service call failed: %s" % e)
    finally:
        node.destroy_node()

def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('spawn_urdf_model_client_node')
    
    service_name = gazebo_namespace + '/spawn_entity'
    node.get_logger().info("Waiting for service %s" % service_name)
    client = node.create_client(SpawnEntity, service_name)
    client.wait_for_service()
    
    try:
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        
        node.get_logger().info("Calling service %s" % service_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()
        
        if resp is not None:
            node.get_logger().info("Spawn status: %s" % resp.status_message)
            return resp.success
        else:
            node.get_logger().error("Service call failed")
            return False
    except Exception as e:
        node.get_logger().error("Service call failed: %s" % e)
        return False
    finally:
        node.destroy_node()

def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('set_model_configuration_client_node')
    
    service_name = gazebo_namespace + '/set_model_configuration'
    node.get_logger().info("Waiting for service %s" % service_name)
    client = node.create_client(SetModelConfiguration, service_name)
    client.wait_for_service()
    
    node.get_logger().info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
    time.sleep(1)
    
    try:
        req = SetModelConfiguration.Request()
        req.model_name = model_name
        req.urdf_param_name = model_param_name
        req.joint_names = joint_names
        req.joint_positions = joint_positions
        
        node.get_logger().info("Calling service %s" % service_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()
        
        if resp is not None:
            node.get_logger().info("Set configuration status: %s" % resp.status_message)
            return resp.success
        else:
            node.get_logger().error("Service call failed")
            return False
    except Exception as e:
        node.get_logger().error("Service call failed: %s" % e)
        return False
    finally:
        node.destroy_node()