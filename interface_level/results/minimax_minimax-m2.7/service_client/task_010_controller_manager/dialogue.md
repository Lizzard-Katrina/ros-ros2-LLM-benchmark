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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ListControllerTypes, ListControllerTypesRequest,
    ListControllers, ListControllersRequest,
    ReloadControllerLibraries, ReloadControllerLibrariesRequest,
    LoadController, LoadControllerRequest,
    UnloadController, UnloadControllerRequest,
    SwitchController, SwitchControllerRequest
)

_node = None

def get_node():
    global _node
    if _node is None:
        rclpy.init()
        _node = rclpy.create_node('controller_manager_tools')
    return _node


def list_controller_types():
    node = get_node()
    node.wait_for_service('controller_manager/list_controller_types')
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    resp = client.call(ListControllerTypesRequest())
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    node = get_node()
    node.wait_for_service('controller_manager/reload_controller_libraries')
    client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    req = ReloadControllerLibrariesRequest(force_kill=force_kill, restore=restore)
    resp = client.call(req)
    return resp.ok


def list_controllers():
    node = get_node()
    node.wait_for_service('controller_manager/list_controllers')
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    resp = client.call(ListControllersRequest())
    for ctrl in resp.controller:
        print(f"Name: {ctrl.name}")
        print(f"  State: {ctrl.state}")
        print(f"  Type: {ctrl.type}")
        claimed = ', '.join(ctrl.claimed_interfaces) if ctrl.claimed_interfaces else 'None'
        print(f"  Claimed interfaces: {claimed}")
        print()


def load_controller(name):
    node = get_node()
    node.wait_for_service('controller_manager/load_controller')
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    resp = client.call(LoadControllerRequest(name))
    if resp.ok:
        print(f"Loaded '{name}'")
        return True
    else:
        print(f"Error when loading '{name}'")
        return False


def unload_controller(name):
    node = get_node()
    node.wait_for_service('controller_manager/unload_controller')
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    resp = client.call(UnloadControllerRequest(name))
    if resp.ok == 1:
        print(f"Unloaded '{name}' successfully")
        return True
    else:
        print(f"Error when unloading '{name}'")
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
    node = get_node()
    node.wait_for_service('controller_manager/switch_controller')
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    req = SwitchControllerRequest(
        start_controllers=start_controllers,
        stop_controllers=stop_controllers,
        strictness=SwitchControllerRequest.STRICT
    )
    resp = client.call(req)
    return resp.ok
```