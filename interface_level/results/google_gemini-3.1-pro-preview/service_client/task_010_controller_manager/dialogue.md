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
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ListControllers,
    LoadController,
    ReloadControllerLibraries,
    SwitchController,
    UnloadController
)


def _call_service(srv_name, srv_type, req):
    initialized = rclpy.ok()
    if not initialized:
        rclpy.init()
    node = rclpy.create_node('controller_manager_client_node')
    client = node.create_client(srv_type, srv_name)
    client.wait_for_service()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    node.destroy_node()
    if not initialized:
        rclpy.shutdown()
    return res


def list_controller_types():
    req = ListControllerTypes.Request()
    resp = _call_service('controller_manager/list_controller_types', ListControllerTypes, req)
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    # TODO: Create the required service clients
    # Perform the reload service call and return a boolean that reflects the service response.
    # If restore is enabled, snapshot controllers before reload and restore their state.
    # END OF TODO
    active_controllers = []
    if restore:
        req_list = ListControllers.Request()
        resp_list = _call_service('controller_manager/list_controllers', ListControllers, req_list)
        active_controllers = [c.name for c in resp_list.controller if c.state == 'active']
        if active_controllers:
            start_stop_controllers(stop_controllers=active_controllers)
            
    req_reload = ReloadControllerLibraries.Request()
    req_reload.force_kill = force_kill
    resp_reload = _call_service('controller_manager/reload_controller_libraries', ReloadControllerLibraries, req_reload)
    
    if restore and active_controllers and resp_reload.ok:
        start_stop_controllers(start_controllers=active_controllers)
        
    return resp_reload.ok


def list_controllers():
    # TODO: 
    # Call the list_controllers service and print a human-readable summary.
    # For each, aggregate claimed hardware interfaces and format output.
    # END OF TODO
    req = ListControllers.Request()
    resp = _call_service('controller_manager/list_controllers', ListControllers, req)
    for c in resp.controller:
        interfaces = ', '.join(c.claimed_interfaces)
        print(f"{c.name} [{c.type}] state: {c.state} claimed interfaces: [{interfaces}]")


def load_controller(name):
    req = LoadController.Request()
    req.name = name
    resp = _call_service('controller_manager/load_controller', LoadController, req)
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    req = UnloadController.Request()
    req.name = name
    resp = _call_service('controller_manager/unload_controller', UnloadController, req)
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
    # TODO: Call the switch_controller service with proper request fields
    # Return a boolean
    # END OF TODO
    req = SwitchController.Request()
    req.activate_controllers = start_controllers
    req.deactivate_controllers = stop_controllers
    req.strictness = SwitchController.Request.STRICT
    resp = _call_service('controller_manager/switch_controller', SwitchController, req)
    return resp.ok
```