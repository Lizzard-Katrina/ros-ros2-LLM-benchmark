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

 #!/usr/bin/env python3
import rclpy
from controller_manager_msgs.srv import ListControllerTypes, ReloadControllerLibraries, ListControllers, LoadController, UnloadController, SwitchController

_node = None

def _get_node():
    global _node
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = rclpy.create_node('controller_manager_interface')
    return _node

def list_controller_types():
    node = _get_node()
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    client.wait_for_service()
    request = ListControllerTypes.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    for t in resp.types:
        print(t)

def reload_libraries(force_kill, restore=False):
    node = _get_node()
    
    running_controllers = []
    if restore:
        list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
        list_client.wait_for_service()
        list_request = ListControllers.Request()
        list_future = list_client.call_async(list_request)
        rclpy.spin_until_future_complete(node, list_future)
        list_resp = list_future.result()
        running_controllers = [c.name for c in list_resp.controller if c.state == 'active']
    
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    reload_client.wait_for_service()
    reload_request = ReloadControllerLibraries.Request()
    reload_request.force_kill = force_kill
    reload_future = reload_client.call_async(reload_request)
    rclpy.spin_until_future_complete(node, reload_future)
    reload_resp = reload_future.result()
    
    if restore and reload_resp.ok:
        for name in running_controllers:
            load_controller(name)
            start_controller(name)
    
    return reload_resp.ok

def list_controllers():
    node = _get_node()
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    client.wait_for_service()
    request = ListControllers.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    
    for c in resp.controller:
        claimed = ', '.join(c.claimed_interfaces) if c.claimed_interfaces else 'none'
        print(f"{c.name} - {c.type} [{c.state}] - {claimed}")

def load_controller(name):
    node = _get_node()
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    client.wait_for_service()
    request = LoadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp.ok:
        print("Loaded '" + name + "'")
        return True
    else:
        print("Error when loading '" + name + "'")
        return False

def unload_controller(name):
    node = _get_node()
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    client.wait_for_service()
    request = UnloadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp.ok:
        print("Unloaded '" + name + "' successfully")
        return True
    else:
        print("Error when unloading '" + name + "'")
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
    node = _get_node()
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    client.wait_for_service()
    request = SwitchController.Request()
    request.start_controllers = start_controllers
    request.stop_controllers = stop_controllers
    request.strictness = 1
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    return resp.ok