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
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ReloadControllerLibraries,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController
)

node = None

def init_node():
    global node
    if node is None:
        rclpy.init()
        node = rclpy.create_node('controller_manager_migration')

def list_controller_types():
    init_node()
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    client.wait_for_service()
    req = ListControllerTypes.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    for t in resp.types:
        print(t)

def reload_libraries(force_kill, restore=False):
    init_node()
    saved_controllers = []
    if restore:
        lc_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
        lc_client.wait_for_service()
        lc_req = ListControllers.Request()
        lc_future = lc_client.call_async(lc_req)
        rclpy.spin_until_future_complete(node, lc_future)
        lc_resp = lc_future.result()
        running_names = []
        for c in lc_resp.controller:
            if c.state == 'running':
                running_names.append(c.name)
        if running_names:
            start_stop_controllers(stop_controllers=running_names)
        for c in lc_resp.controller:
            if c.state != 'unloaded':
                unload_controller(c.name)
        saved_controllers = lc_resp.controller

    rc_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    rc_client.wait_for_service()
    rc_req = ReloadControllerLibraries.Request()
    rc_req.force_kill = force_kill
    rc_future = rc_client.call_async(rc_req)
    rclpy.spin_until_future_complete(node, rc_future)
    success = rc_future.result().ok

    if restore and success:
        for c in saved_controllers:
            load_controller(c.name)
        running_to_restore = [c.name for c in saved_controllers if c.state == 'running']
        if running_to_restore:
            start_controllers(running_to_restore)

    return success

def list_controllers():
    init_node()
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    client.wait_for_service()
    req = ListControllers.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    for c in resp.controller:
        claimed = " ".join(c.claimed_interfaces)
        print(f"Controller: {c.name}, State: {c.state}, Type: {c.type}, Hardware: {claimed}")

def load_controller(name):
    init_node()
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    client.wait_for_service()
    req = LoadController.Request()
    req.name = name
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp.ok:
        print("Loaded '" + name + "'")
        return True
    else:
        print("Error when loading '" + name + "'")
        return False

def unload_controller(name):
    init_node()
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    client.wait_for_service()
    req = UnloadController.Request()
    req.name = name
    future = client.call_async(req)
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
    init_node()
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    client.wait_for_service()
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = 2
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    return resp.ok