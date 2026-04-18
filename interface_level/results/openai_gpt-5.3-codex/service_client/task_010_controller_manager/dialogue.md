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

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ReloadControllerLibraries,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController,
)

_node = None


def _get_node():
    global _node
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = Node("controller_manager_interface")
    return _node


def _call_service(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(_get_node(), future)
    if future.result() is None:
        raise RuntimeError("Service call failed")
    return future.result()


def _create_client(srv_type, srv_name):
    node = _get_node()
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            raise RuntimeError(f"Interrupted while waiting for service {srv_name}")
    return client


def list_controller_types():
    s = _create_client(ListControllerTypes, "controller_manager/list_controller_types")
    resp = _call_service(s, ListControllerTypes.Request())
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    reload_client = _create_client(
        ReloadControllerLibraries, "controller_manager/reload_controller_libraries"
    )

    snapshot = []
    active_before = []
    if restore:
        list_client = _create_client(ListControllers, "controller_manager/list_controllers")
        list_resp = _call_service(list_client, ListControllers.Request())
        snapshot = [c.name for c in list_resp.controller]
        active_before = [
            c.name for c in list_resp.controller if c.state in ("active", "running")
        ]

    req = ReloadControllerLibraries.Request()
    req.force_kill = force_kill
    resp = _call_service(reload_client, req)

    if not resp.ok:
        return False

    if restore:
        ok = True
        for name in snapshot:
            if not load_controller(name):
                ok = False
        if active_before and not start_controllers(active_before):
            ok = False
        return ok

    return True


def list_controllers():
    client = _create_client(ListControllers, "controller_manager/list_controllers")
    resp = _call_service(client, ListControllers.Request())

    for c in resp.controller:
        claimed = sorted(set(c.claimed_interfaces)) if c.claimed_interfaces else []
        claimed_str = ", ".join(claimed) if claimed else "none"
        print(f"{c.name}: state={c.state}, type={c.type}, claimed_interfaces=[{claimed_str}]")


def load_controller(name):
    s = _create_client(LoadController, "controller_manager/load_controller")
    req = LoadController.Request()
    req.name = name
    resp = _call_service(s, req)
    if resp.ok:
        print("Loaded '" + name + "'")
        return True
    else:
        print("Error when loading '" + name + "'")
        return False


def unload_controller(name):
    s = _create_client(UnloadController, "controller_manager/unload_controller")
    req = UnloadController.Request()
    req.name = name
    resp = _call_service(s, req)
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
    s = _create_client(SwitchController, "controller_manager/switch_controller")
    req = SwitchController.Request()
    req.activate_controllers = list(start_controllers)
    req.deactivate_controllers = list(stop_controllers)
    req.strictness = SwitchController.Request.STRICT
    req.activate_asap = True
    req.timeout = Duration(sec=5, nanosec=0)

    resp = _call_service(s, req)
    return bool(resp.ok)