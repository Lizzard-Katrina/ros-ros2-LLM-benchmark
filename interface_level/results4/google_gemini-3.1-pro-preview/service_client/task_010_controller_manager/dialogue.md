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

FILE_PATH: controller_manager_interface.py
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

[FILENAME: controller_manager_interface.py]
#! /usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController,
    ReloadControllerLibraries
)

def _get_node():
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node('controller_manager_interface_node')

def _call_service(node, srv_type, srv_name, request):
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        pass
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

def list_controller_types():
    node = _get_node()
    req = ListControllerTypes.Request()
    resp = _call_service(node, ListControllerTypes, 'controller_manager/list_controller_types', req)
    for t in resp.types:
        print(t)
    node.destroy_node()

def reload_libraries(force_kill, restore = False):
    node = _get_node()
    running_controllers = []
    
    if restore:
        req_list = ListControllers.Request()
        resp_list = _call_service(node, ListControllers, 'controller_manager/list_controllers', req_list)
        running_controllers = [c.name for c in resp_list.controller if c.state == 'active']
        
        if running_controllers:
            req_stop = SwitchController.Request()
            req_stop.stop_controllers = running_controllers
            req_stop.strictness = SwitchController.Request.BEST_EFFORT
            _call_service(node, SwitchController, 'controller_manager/switch_controller', req_stop)
            
    req = ReloadControllerLibraries.Request()
    req.force_kill = force_kill
    resp = _call_service(node, ReloadControllerLibraries, 'controller_manager/reload_controller_libraries', req)
    
    if restore and running_controllers:
        req_start = SwitchController.Request()
        req_start.start_controllers = running_controllers
        req_start.strictness = SwitchController.Request.BEST_EFFORT
        _call_service(node, SwitchController, 'controller_manager/switch_controller', req_start)
        
    node.destroy_node()
    return resp.ok

def list_controllers():
    node = _get_node()
    req = ListControllers.Request()
    resp = _call_service(node, ListControllers, 'controller_manager/list_controllers', req)
    for c in resp.controller:
        interfaces = ', '.join(c.claimed_interfaces)
        print(f"{c.name} - {c.type} (state: {c.state}) - claimed interfaces: [{interfaces}]")
    node.destroy_node()

def load_controller(name):
    node = _get_node()
    req = LoadController.Request()
    req.name = name
    resp = _call_service(node, LoadController, 'controller_manager/load_controller', req)
    node.destroy_node()
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False

def unload_controller(name):
    node = _get_node()
    req = UnloadController.Request()
    req.name = name
    resp = _call_service(node, UnloadController, 'controller_manager/unload_controller', req)
    node.destroy_node()
    if resp.ok:
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
    node = _get_node()
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.STRICT
    resp = _call_service(node, SwitchController, 'controller_manager/switch_controller', req)
    node.destroy_node()
    return resp.ok