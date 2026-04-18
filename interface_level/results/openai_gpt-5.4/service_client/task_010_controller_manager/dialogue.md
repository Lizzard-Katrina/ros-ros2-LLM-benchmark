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

#! /usr/bin/env python3
from __future__ import print_function

import time

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import (
    ListControllerTypes,
    ListControllers,
    ListControllerTypesRequest,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
    UnloadController,
    UnloadControllerRequest,
    ReloadControllerLibraries,
    ReloadControllerLibrariesRequest,
    SwitchController,
    SwitchControllerRequest,
)


class ControllerManagerClient(Node):
    def __init__(self):
        super().__init__('controller_manager_client')


_node = None


def _get_node():
    global _node
    if _node is None:
        rclpy.init(args=None)
        _node = ControllerManagerClient()
    return _node


def _call_service(service_name, srv_type, request):
    node = _get_node()
    client = node.create_client(srv_type, service_name)
    while not client.wait_for_service(timeout_sec=1.0):
        pass
    future = client.call_async(request)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.result()


def list_controller_types():
    resp = _call_service(
        'controller_manager/list_controller_types',
        ListControllerTypes,
        ListControllerTypesRequest()
    )
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    node = _get_node()

    controllers_snapshot = []
    running_before = []

    if restore:
        list_resp = _call_service(
            'controller_manager/list_controllers',
            ListControllers,
            ListControllersRequest()
        )
        controllers_snapshot = list(list_resp.controller)
        running_before = [c.name for c in controllers_snapshot if c.state == 'running']

    reload_resp = _call_service(
        'controller_manager/reload_controller_libraries',
        ReloadControllerLibraries,
        ReloadControllerLibrariesRequest(force_kill=force_kill)
    )

    if not reload_resp.ok:
        return False

    if restore:
        for c in controllers_snapshot:
            load_controller(c.name)

        if running_before:
            start_controllers(running_before)

    return bool(reload_resp.ok)


def list_controllers():
    resp = _call_service(
        'controller_manager/list_controllers',
        ListControllers,
        ListControllersRequest()
    )

    for c in resp.controller:
        interfaces = []
        for res in c.claimed_resources:
            for iface in res.resources:
                interfaces.append(iface)
        interfaces_str = ', '.join(interfaces) if interfaces else 'none'
        ctype = c.type if c.type else 'unknown'
        print(f"{c.name} - state: {c.state}, type: {ctype}, claimed interfaces: {interfaces_str}")


def load_controller(name):
    resp = _call_service(
        'controller_manager/load_controller',
        LoadController,
        LoadControllerRequest(name=name)
    )
    if resp.ok:
        print("Loaded '" + name + "'")
        return True
    else:
        print("Error when loading '" + name + "'")
        return False


def unload_controller(name):
    resp = _call_service(
        'controller_manager/unload_controller',
        UnloadController,
        UnloadControllerRequest(name=name)
    )
    if resp.ok == 1:
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
    req = SwitchControllerRequest()
    req.start_controllers = list(start_controllers)
    req.stop_controllers = list(stop_controllers)
    req.strictness = SwitchControllerRequest.STRICT
    req.start_asap = False
    req.timeout = 0.0

    resp = _call_service(
        'controller_manager/switch_controller',
        SwitchController,
        req
    )
    return bool(resp.ok)