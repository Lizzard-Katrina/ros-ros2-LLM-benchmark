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
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, ListControllerTypesRequest, ReloadControllerLibraries, ReloadControllerLibrariesRequest, ListControllers, ListControllersRequest, LoadController, LoadControllerRequest, UnloadController, UnloadControllerRequest, SwitchController, SwitchControllerRequest

_node = None

def _get_node():
    global _node
    if _node is None:
        rclpy.init()
        _node = Node('controller_manager_interface')
    return _node

def list_controller_types():
    node = _get_node()
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/list_controller_types...')
    req = ListControllerTypesRequest()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore = False):
    node = _get_node()
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    switch_client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    while not reload_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/reload_controller_libraries...')
    while not list_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/list_controllers...')
    while not switch_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/switch_controller...')

    snapshot = []
    if restore:
        list_req = ListControllersRequest()
        list_future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, list_future)
        list_resp = list_future.result()
        for c in list_resp.controller:
            if c.state == 'active':
                snapshot.append(c.name)
        if snapshot:
            stop_req = SwitchControllerRequest()
            stop_req.stop_controllers = snapshot
            stop_req.strictness = SwitchControllerRequest.BEST_EFFORT
            stop_req.start_asynchronously = False
            stop_future = switch_client.call_async(stop_req)
            rclpy.spin_until_future_complete(node, stop_future)

    req = ReloadControllerLibrariesRequest()
    req.force_kill = force_kill
    future = reload_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()

    if restore and snapshot and resp.ok:
        start_req = SwitchControllerRequest()
        start_req.start_controllers = snapshot
        start_req.strictness = SwitchControllerRequest.BEST_EFFORT
        start_req.start_asynchronously = False
        start_future = switch_client.call_async(start_req)
        rclpy.spin_until_future_complete(node, start_future)

    return resp.ok

def list_controllers():
    node = _get_node()
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/list_controllers...')
    req = ListControllersRequest()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    for c in resp.controller:
        claimed_interfaces = ", ".join(c.claimed_interfaces) if c.claimed_interfaces else "none"
        print("name: {}, state: {}, type: {}, claimed interfaces: [{}]".format(c.name, c.state, c.type, claimed_interfaces))

def load_controller(name):
    node = _get_node()
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/load_controller...')
    req = LoadControllerRequest(name=name)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    node = _get_node()
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/unload_controller...')
    req = UnloadControllerRequest(name=name)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
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
    node = _get_node()
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Waiting for service controller_manager/switch_controller...')
    req = SwitchControllerRequest()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchControllerRequest.BEST_EFFORT
    req.start_asynchronously = False
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    return resp.ok