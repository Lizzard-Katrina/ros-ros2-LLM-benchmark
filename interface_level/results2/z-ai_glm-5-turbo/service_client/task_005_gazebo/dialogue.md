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

FILE_PATH: gazebo_interface.py
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

[FILENAME: gazebo_interface.py]
#! /usr/bin/env python3
# Wrappers around the services provided by rosified gazebo

import sys
import rclpy
import os
import time

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench


def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/spawn_sdf_model"%gazebo_namespace)
    node = rclpy.create_node('gazebo_interface_spawn_sdf')
    client = node.create_client(SpawnModel, gazebo_namespace+'/spawn_sdf_model')
    while not client.wait_for_service(timeout_sec=1.0):
        rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/spawn_sdf_model"%gazebo_namespace)
    try:
      req = SpawnModel.Request()
      req.model_name = model_name
      req.model_xml = model_xml
      req.robot_namespace = robot_namespace
      req.initial_pose = initial_pose
      req.reference_frame = reference_frame
      rclpy.logging.get_logger('gazebo_interface').info("Calling service %s/spawn_sdf_model"%gazebo_namespace)
      future = client.call_async(req)
      rclpy.spin_until_future_complete(node, future)
      resp = future.result()
      rclpy.logging.get_logger('gazebo_interface').info("Spawn status: %s"%resp.status_message)
      node.destroy_node()
      return resp.success
    except Exception as e:
      print("Service call failed: %s" % e)
      node.destroy_node()

def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/spawn_urdf_model"%gazebo_namespace)
    node = rclpy.create_node('gazebo_interface_spawn_urdf')
    client = node.create_client(SpawnModel, gazebo_namespace+'/spawn_urdf_model')
    while not client.wait_for_service(timeout_sec=1.0):
        rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/spawn_urdf_model"%gazebo_namespace)
    try:
      req = SpawnModel.Request()
      req.model_name = model_name
      req.model_xml = model_xml
      req.robot_namespace = robot_namespace
      req.initial_pose = initial_pose
      req.reference_frame = reference_frame
      rclpy.logging.get_logger('gazebo_interface').info("Calling service %s/spawn_urdf_model"%gazebo_namespace)
      future = client.call_async(req)
      rclpy.spin_until_future_complete(node, future)
      resp = future.result()
      rclpy.logging.get_logger('gazebo_interface').info("Spawn status: %s"%resp.status_message)
      node.destroy_node()
      return resp.success
    except Exception as e:
      print("Service call failed: %s" % e)
      node.destroy_node()

def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    node = rclpy.create_node('gazebo_interface_set_config')
    client = node.create_client(SetModelConfiguration, gazebo_namespace+'/set_model_configuration')
    while not client.wait_for_service(timeout_sec=1.0):
        rclpy.logging.get_logger('gazebo_interface').info("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    rclpy.logging.get_logger('gazebo_interface').info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.");
    time.sleep(1)
    try:
      req = SetModelConfiguration.Request()
      req.model_name = model_name
      req.model_param_name = model_param_name
      req.joint_names = joint_names
      req.joint_positions = joint_positions
      rclpy.logging.get_logger('gazebo_interface').info("Calling service %s/set_model_configuration"%gazebo_namespace)
      future = client.call_async(req)
      rclpy.spin_until_future_complete(node, future)
      resp = future.result()
      rclpy.logging.get_logger('gazebo_interface').info("Set model configuration status: %s"%resp.success)
      node.destroy_node()
      return resp.success
    except Exception as e:
      print("Service call failed: %s" % e)
      node.destroy_node()