# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import rospy
import os
import time

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/spawn_sdf_model"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/spawn_sdf_model')
    try:
      spawn_sdf_model = rospy.ServiceProxy(gazebo_namespace+'/spawn_sdf_model', SpawnModel)
      rospy.loginfo("Calling service %s/spawn_sdf_model"%gazebo_namespace)
      resp = spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
      rospy.loginfo("Spawn status: %s"%resp.status_message)
      return resp.success
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)

def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/spawn_urdf_model"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/spawn_urdf_model')
    # TODO:
    # - Implement the client-side logic for spawning a URDF model in Gazebo
    # - Make sure to interact with the correct ROS service using the function's inputs
    # - Capture the result of the service call and return some indication of success
    # - Log or display the outcome
    #END OF TODO

def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/set_model_configuration')
    rospy.loginfo("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.");
    time.sleep(1)
    # TODO:
    # - Implement the client-side logic for setting a model's joint configuration
    # - Use the function arguments to prepare a request for the appropriate service
    # - Capture and report the result in some form
    # - Ensure the service call outcome is communicated (logging, returning status, etc.)
    # END OF TODO

----------------------------


# LLM Response

#! /usr/bin/env python3
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
    if _node is None:
        rclpy.init(args=None)
        _node = Node('gazebo_ros2_client')
    return _node


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = _get_node()
    service_name = gazebo_namespace + '/spawn_sdf_model'
    node.get_logger().info("Waiting for service %s" % service_name)
    client = node.create_client(SpawnEntity, service_name)
    client.wait_for_service()

    req = SpawnEntity.Request()
    req.name = model_name
    req.xml = model_xml
    req.robot_namespace = robot_namespace
    req.initial_pose = initial_pose
    req.reference_frame = reference_frame

    try:
        node.get_logger().info("Calling service %s" % service_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()
        node.get_logger().info("Spawn status: %s" % resp.status_message)
        return resp.success
    except Exception as e:
        print("Service call failed: %s" % e)


def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node = _get_node()
    service_name = gazebo_namespace + '/spawn_urdf_model'
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
        node.get_logger().info("Spawn status: %s" % resp.status_message)
        return resp.success
    except Exception as e:
        print("Service call failed: %s" % e)


def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    node = _get_node()
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
        node.get_logger().info("Set model configuration status: %s" % resp.status_message)
        return resp.success
    except Exception as e:
        print("Service call failed: %s" % e)