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
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController,
    ReloadControllerLibraries,
)


_node = None


def _get_node():
    global _node
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = rclpy.create_node('controller_manager_interface')
    return _node


def _call_service(service_name, service_type, request):
    node = _get_node()
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("Service '{}' is not available".format(service_name))

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        exc = future.exception()
        if exc is not None:
            raise exc
        raise RuntimeError("Service call to '{}' failed".format(service_name))

    return future.result()


def list_controller_types():
    req = ListControllerTypes.Request()
    resp = _call_service('controller_manager/list_controller_types', ListControllerTypes, req)
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    controllers_snapshot = []

    if restore:
        try:
            list_req = ListControllers.Request()
            list_resp = _call_service('controller_manager/list_controllers', ListControllers, list_req)
            controllers_snapshot = [
                (c.name, c.state)
                for c in list_resp.controller
            ]
        except Exception as exc:
            print("Error when listing controllers before reload: {}".format(exc))
            return False

    try:
        req = ReloadControllerLibraries.Request()
        req.force_kill = bool(force_kill)
        resp = _call_service(
            'controller_manager/reload_controller_libraries',
            ReloadControllerLibraries,
            req
        )
    except Exception as exc:
        print("Error when reloading controller libraries: {}".format(exc))
        return False

    if not resp.ok:
        print("Error when reloading controller libraries")
        return False

    print("Reloaded controller libraries")

    if restore:
        ok = True
        for name, _state in controllers_snapshot:
            if not load_controller(name):
                ok = False

        start_names = [
            name for name, state in controllers_snapshot
            if state == 'running' or state == 'active'
        ]

        if start_names:
            ok = start_stop_controllers(start_controllers=start_names) and ok

        return ok

    return True


def list_controllers():
    try:
        req = ListControllers.Request()
        resp = _call_service('controller_manager/list_controllers', ListControllers, req)
    except Exception as exc:
        print("Error when listing controllers: {}".format(exc))
        return False

    for c in resp.controller:
        print("{} ({}) - {}".format(c.name, c.type, c.state))

        claimed = {}
        for resource in c.claimed_resources:
            interface = getattr(resource, 'hardware_interface', '')
            resources = list(getattr(resource, 'resources', []))
            if interface not in claimed:
                claimed[interface] = []
            claimed[interface].extend(resources)

        if claimed:
            for interface in sorted(claimed.keys()):
                resources = sorted(set(claimed[interface]))
                print("  {}: {}".format(interface, ', '.join(resources)))
        else:
            print("  claimed resources: none")

    return True


def load_controller(name):
    req = LoadController.Request()
    req.name = name
    resp = _call_service('controller_manager/load_controller', LoadController, req)
    if resp.ok:
        print("Loaded '" + name + "'")
        return True
    else:
        print("Error when loading '" + name + "'")
        return False


def unload_controller(name):
    req = UnloadController.Request()
    req.name = name
    resp = _call_service('controller_manager/unload_controller', UnloadController, req)
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
    req = SwitchController.Request()
    req.start_controllers = list(start_controllers)
    req.stop_controllers = list(stop_controllers)

    if hasattr(req, 'strictness'):
        req.strictness = SwitchController.Request.STRICT

    if hasattr(req, 'start_asap'):
        req.start_asap = False

    if hasattr(req, 'activate_asap'):
        req.activate_asap = False

    if hasattr(req, 'timeout'):
        req.timeout.sec = 0
        req.timeout.nanosec = 0

    try:
        resp = _call_service('controller_manager/switch_controller', SwitchController, req)
    except Exception as exc:
        print("Error when switching controllers: {}".format(exc))
        return False

    if resp.ok:
        if start_controllers:
            print("Started controllers: " + ", ".join(start_controllers))
        if stop_controllers:
            print("Stopped controllers: " + ", ".join(stop_controllers))
        return True
    else:
        print("Error when switching controllers")
        return False