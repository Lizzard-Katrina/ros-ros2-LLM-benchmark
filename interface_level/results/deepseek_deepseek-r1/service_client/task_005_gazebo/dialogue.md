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

```python
#!/usr/bin/env python3
# Wrappers around the services provided by rosified gazebo

import sys
import os
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration
from geometry_msgs.msg import Pose


def spawn_sdf_model_client(node, model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node.get_logger().info(f"Waiting for service {gazebo_namespace}/spawn_sdf_model")
    
    client = node.create_client(SpawnModel, gazebo_namespace + '/spawn_sdf_model')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service {gazebo_namespace}/spawn_sdf_model not available")
        return False
    
    try:
        req = SpawnModel.Request()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        
        node.get_logger().info(f"Calling service {gazebo_namespace}/spawn_sdf_model")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            resp = future.result()
            node.get_logger().info(f"Spawn status: {resp.status_message}")
            return resp.success
        else:
            node.get_logger().error(f"Service call failed: {future.exception()}")
            return False
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")
        return False


def spawn_urdf_model_client(node, model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    node.get_logger().info(f"Waiting for service {gazebo_namespace}/spawn_urdf_model")
    
    client = node.create_client(SpawnModel, gazebo_namespace + '/spawn_urdf_model')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service {gazebo_namespace}/spawn_urdf_model not available")
        return False
    
    try:
        req = SpawnModel.Request()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        
        node.get_logger().info(f"Calling service {gazebo_namespace}/spawn_urdf_model")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            resp = future.result()
            node.get_logger().info(f"Spawn status: {resp.status_message}")
            return resp.success
        else:
            node.get_logger().error(f"Service call failed: {future.exception()}")
            return False
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")
        return False


def set_model_configuration_client(node, model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    node.get_logger().info(f"Waiting for service {gazebo_namespace}/set_model_configuration")
    
    client = node.create_client(SetModelConfiguration, gazebo_namespace + '/set_model_configuration')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service {gazebo_namespace}/set_model_configuration not available")
        return False
    
    node.get_logger().info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
    time.sleep(1)
    
    try:
        req = SetModelConfiguration.Request()
        req.model_name = model_name
        req.urdf_param_name = model_param_name
        req.joint_names = joint_names
        req.joint_positions = joint_positions
        
        node.get_logger().info(f"Calling service {gazebo_namespace}/set_model_configuration")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            resp = future.result()
            node.get_logger().info(f"Set configuration status: {resp.status_message}")
            return resp.success
        else:
            node.get_logger().error(f"Service call failed: {future.exception()}")
            return False
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")
        return False
```