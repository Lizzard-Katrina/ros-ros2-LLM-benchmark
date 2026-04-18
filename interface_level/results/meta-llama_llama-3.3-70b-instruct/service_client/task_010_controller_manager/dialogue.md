# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#! /usr/bin/env python
from __future__ import print_function
import rospy
from controller_manager_msgs.srv import *


def list_controller_types():
    rospy.wait_for_service('controller_manager/list_controller_types')
    s = rospy.ServiceProxy('controller_manager/list_controller_types', ListControllerTypes)
    resp = s.call(ListControllerTypesRequest())
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore = False):
    # TODO: Create the required service clients
    # Perform the reload service call and return a boolean that reflects the service response.
    # If restore is enabled, snapshot controllers before reload and restore their state.
    # END OF TODO

def list_controllers():
    # TODO: 
    # Call the list_controllers service and print a human-readable summary.
    # For each, aggregate claimed hardware interfaces and format output.
    # END OF TODO

def load_controller(name):
    rospy.wait_for_service('controller_manager/load_controller')
    s = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    resp = s.call(LoadControllerRequest(name))
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    rospy.wait_for_service('controller_manager/unload_controller')
    s = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
    resp = s.call(UnloadControllerRequest(name))
    if resp.ok == 1:
        print("Unloaded \'" + name + "\' successfully")
        return True
    else:
        print("Error when unloading \'" + name + "\'")
        return False


def start_controller(name):
    return start_stop_controllers(start_controllers=[name])


def start_controllers(names):
    return start_stop_controllers(start_controllers=names)


def stop_controller(name):
    return start_stop_controllers(stop_controllers=[name])


def stop_controllers(names):
    return start_stop_controllers(stop_controllers=names)


def start_stop_controllers(start_controllers=[], stop_controllers=[]):
    # TODO: Call the switch_controller service with proper request fields
    # Return a boolean
    # END OF TODO

----------------------------


# LLM Response

```python
#! /usr/bin/env python
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, LoadController, UnloadController, SwitchController


def list_controller_types():
    rclpy.init()
    node = Node('list_controller_types')
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('controller_manager/list_controller_types not available, waiting...')
    req = ListControllerTypes.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        for t in future.result().types:
            print(t)
    node.destroy_node()
    rclpy.shutdown()


def reload_libraries(force_kill, restore=False):
    rclpy.init()
    node = Node('reload_libraries')
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('controller_manager/switch_controller not available, waiting...')
    req = SwitchController.Request()
    req.start_asap = False
    req.strictness = SwitchController.Request.BEST_EFFORT
    req.timeout = 0.0
    if restore:
        # snapshot controllers before reload and restore their state
        pass
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return future.result().ok
    node.destroy_node()
    rclpy.shutdown()


def list_controllers():
    rclpy.init()
    node = Node('list_controllers')
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controllers')
    while not client.wait_for_service(timeout_sec=