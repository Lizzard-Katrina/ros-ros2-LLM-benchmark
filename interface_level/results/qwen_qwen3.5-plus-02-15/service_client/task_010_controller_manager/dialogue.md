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


</think>

#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import atexit
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ReloadControllerLibraries,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController
)

rclpy.init()
node = Node('controller_manager_client')
atexit.register(rclpy.shutdown)


def _call_service(client_type, service_name, request):
    client = node.create_client(client_type, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"Service {service_name} not available")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.exception() is not None:
        print(f"Service call failed: {future.exception()}")
        return None
    return future.result()


def list_controller_types():
    req = ListControllerTypes.Request()
    resp = _call_service(ListControllerTypes, '/controller_manager/list_controller_types', req)
    if resp:
        for t in resp.types:
            print(t)


def reload_libraries(force_kill, restore=False):
    req = ReloadControllerLibraries.Request()
    req.restore = restore
    # force_kill argument is kept for signature compatibility but not used in standard ROS2 service
    resp = _call_service(ReloadControllerLibraries, '/controller_manager/reload_controller_libraries', req)
    if resp:
        if resp.ok:
            print("Reloaded libraries successfully")
            return True
        else:
            print("Error when reloading libraries")
            return False
    return False


def list_controllers():
    req = ListControllers.Request()
    resp = _call_service(ListControllers, '/controller_manager/list_controllers', req)
    if resp:
        for c in resp.controller:
            resources = ', '.join(c.claimed_resources) if c.claimed_resources else 'none'
            print(f"{c.name} ({c.state}): {resources}")


def load_controller(name):
    req = LoadController.Request()
    req.name = name
    resp = _call_service(LoadController, '/controller_manager/load_controller', req)
    if resp:
        if resp.ok:
            print(f"Loaded '{name}'")
            return True
        else:
            print(f"Error when loading '{name}'")
            return False
    return False


def unload_controller(name):
    req = UnloadController.Request()
    req.name = name
    resp = _call_service(UnloadController, '/controller_manager/unload_controller', req)
    if resp:
        if resp.ok:
            print(f"Unloaded '{name}' successfully")
            return True
        else:
            print(f"Error when unloading '{name}'")
            return False
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
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = 1  # STRICT
    req.timeout = Duration(seconds=0.0)
    
    resp = _call_service(SwitchController, '/controller_manager/switch_controller', req)
    if resp:
        return bool(resp.ok)
    return False