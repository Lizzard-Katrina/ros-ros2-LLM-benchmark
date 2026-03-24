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
#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration
from geometry_msgs.msg import Pose

class GazeboClient(Node):

    def __init__(self):
        super().__init__('gazebo_client')

    def spawn_sdf_model_client(self, model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
        self.get_logger().info("Waiting for service %s/spawn_sdf_model" % gazebo_namespace)
        self.create_service(SpawnModel, gazebo_namespace + '/spawn_sdf_model', self.spawn_sdf_model_callback)
        self.wait_for_service(gazebo_namespace + '/spawn_sdf_model')
        try:
            spawn_sdf_model = self.create_client(SpawnModel, gazebo_namespace + '/spawn_sdf_model')
            self.get_logger().info("Calling service %s/spawn_sdf_model" % gazebo_namespace)
            request = SpawnModel.Request()
            request.model_name = model_name
            request.model_xml = model_xml
            request.robot_namespace = robot_namespace
            request.initial_pose = initial_pose
            request.reference_frame = reference_frame
            future = spawn_sdf_model.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            self.get_logger().info("Spawn status: %s" % resp.status_message)
            return resp.success
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)

    def spawn_urdf_model_client(self, model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
        self.get_logger().info("Waiting for service %s/spawn_urdf_model" % gazebo_namespace)
        self.wait_for_service(gazebo_namespace + '/spawn_urdf_model')
        try:
            spawn_urdf_model = self.create_client(SpawnModel, gazebo_namespace + '/spawn_urdf_model')
            self.get_logger().info("Calling service %s/spawn_urdf_model" % gazebo_namespace)
            request = SpawnModel.Request()
            request.model_name = model_name
            request.model_xml = model_xml
            request.robot_namespace = robot_namespace
            request.initial_pose = initial_pose
            request.reference_frame = reference_frame
            future = spawn_urdf_model.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            self.get_logger().info("Spawn status: %s" % resp.status_message)
            return resp.success
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)

    def set_model_configuration_client(self, model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
        self.get_logger().info("Waiting for service %s/set_model_configuration" % gazebo_namespace)
        self.wait_for_service(gazebo_namespace + '/set_model_configuration')
        self.get_logger().info("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.")
        time.sleep(1)
        try:
            set_model_configuration = self.create_client(SetModelConfiguration, gazebo_namespace + '/set_model_configuration')
            self.get_logger().info("Calling service %s/set_model_configuration" % gazebo_namespace)
            request = SetModelConfiguration.Request()
            request.model_name = model_name
            request.urdf_param_name = model_param_name
            request.joint_names = joint_names
            request.joint_positions = joint_positions
            future = set_model_configuration.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            self.get_logger().info("Set model configuration status: %s" % resp.success)
            return resp.success
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)

def main(args=None):
    rclpy.init(args=args)
    gazebo_client = GazeboClient()
    rclpy.spin(gazebo_client)
    gazebo_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```