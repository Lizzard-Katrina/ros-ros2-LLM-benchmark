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
#! /usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import *


node = None


def get_node():
    global node
    if node is None:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node('controller_manager_client')
    return node


def wait_and_call(srv_name, srv_type, request):
    n = get_node()
    client = n.create_client(srv_type, srv_name)
    client.wait_for_service()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)
    return future.result()


def list_controller_types():
    resp = wait_and_call(
        'controller_manager/list_controller_types',
        ListControllerTypes,
        ListControllerTypes.Request()
    )
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    n = get_node()

    if restore:
        list_client = n.create_client(ListControllers, 'controller_manager/list_controllers')
        list_client.wait_for_service()
        future = list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(n, future)
        list_resp = future.result()

        started_controllers = []
        stopped_controllers = []
        for c in list_resp.controller:
            if c.state == 'running':
                started_controllers.append(c.name)
            else:
                stopped_controllers.append(c.name)

    req = ReloadControllerLibraries.Request()
    req.force_kill = force_kill
    resp = wait_and_call(
        'controller_manager/reload_controller_libraries',
        ReloadControllerLibraries,
        req
    )

    if not resp.ok:
        print("Error when reloading libraries")
        return False

    print("Reloaded libraries successfully")

    if restore:
        for name in started_controllers + stopped_controllers:
            load_controller(name)
        if started_controllers:
            start_stop_controllers(start_controllers=started_controllers)

    return True


def list_controllers():
    resp = wait_and_call(
        'controller_manager/list_controllers',
        ListControllers,
        ListControllers.Request()
    )

    if len(resp.controller) == 0:
        print("No controllers are loaded in mechanism control")
    else:
        for c in resp.controller:
            hw_ifaces = []
            if hasattr(c, 'claimed_resources'):
                for resource in c.claimed_resources:
                    hw_ifaces.append(resource.hardware_interface)
            hw_iface_str = ', '.join(hw_ifaces) if hw_ifaces else ''

            state_str = c.state if hasattr(c, 'state') else 'unknown'
            type_str = c.type if hasattr(c, 'type') else 'unknown'

            if hw_iface_str:
                print(" - %s (%s)\n\tState: %s\n\tHardware Interfaces: %s" %
                      (c.name, type_str, state_str, hw_iface_str))
            else:
                print(" - %s (%s)\n\tState: %s" %
                      (c.name, type_str, state_str))


def load_controller(name):
    req = LoadController.Request()
    req.name = name
    resp = wait_and_call(
        'controller_manager/load_controller',
        LoadController,
        req
    )
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    req = UnloadController.Request()
    req.name = name
    resp = wait_and_call(
        'controller_manager/unload_controller',
        UnloadController,
        req
    )
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
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.STRICT

    resp = wait_and_call(
        'controller_manager/switch_controller',
        SwitchController,
        req
    )

    if resp.ok:
        return True
    else:
        print("Error when switching controllers")
        return False
```