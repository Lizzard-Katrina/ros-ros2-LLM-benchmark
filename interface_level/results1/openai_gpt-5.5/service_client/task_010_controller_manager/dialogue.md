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

import sys
import time

import rclpy
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
        _node = rclpy.create_node('controller_manager_interface')
    return _node


def _service_name(name):
    return '/' + name.lstrip('/')


def _wait_for_service(client, service_name):
    node = _get_node()
    while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service %s not available, waiting again...' % service_name)


def _call_service(service_name, srv_type, request):
    node = _get_node()
    client = node.create_client(srv_type, _service_name(service_name))
    _wait_for_service(client, service_name)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        exc = future.exception()
        if exc is not None:
            raise exc
        raise RuntimeError('Service call failed: %s' % service_name)

    return future.result()


def _strict_value():
    if hasattr(SwitchController.Request, 'STRICT'):
        return SwitchController.Request.STRICT
    return 2


def _best_effort_value():
    if hasattr(SwitchController.Request, 'BEST_EFFORT'):
        return SwitchController.Request.BEST_EFFORT
    return 1


def list_controller_types():
    resp = _call_service(
        'controller_manager/list_controller_types',
        ListControllerTypes,
        ListControllerTypes.Request(),
    )
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    list_resp = None
    running_controllers = []

    if restore:
        try:
            list_resp = _call_service(
                'controller_manager/list_controllers',
                ListControllers,
                ListControllers.Request(),
            )
            for controller in list_resp.controller:
                if controller.state == 'running' or controller.state == 'active':
                    running_controllers.append(controller.name)
        except Exception as exc:
            print('Error when listing controllers before reload: %s' % exc)
            return False

    req = ReloadControllerLibraries.Request()
    req.force_kill = bool(force_kill)

    try:
        resp = _call_service(
            'controller_manager/reload_controller_libraries',
            ReloadControllerLibraries,
            req,
        )
    except Exception as exc:
        print('Error when reloading controller libraries: %s' % exc)
        return False

    if not resp.ok:
        print('Error when reloading controller libraries')
        return False

    print('Reloaded controller libraries successfully')

    if restore and list_resp is not None:
        for controller in list_resp.controller:
            try:
                load_controller(controller.name)
            except Exception:
                pass

        if running_controllers:
            if not start_stop_controllers(start_controllers=running_controllers):
                print('Error when restoring controller state')
                return False

    return True


def list_controllers():
    resp = _call_service(
        'controller_manager/list_controllers',
        ListControllers,
        ListControllers.Request(),
    )

    for c in resp.controller:
        claimed = []
        resources = getattr(c, 'claimed_resources', [])
        for resource in resources:
            interface = getattr(resource, 'hardware_interface', '')
            resource_names = list(getattr(resource, 'resources', []))
            if resource_names:
                claimed.append('%s: %s' % (interface, ', '.join(resource_names)))
            elif interface:
                claimed.append(interface)

        claimed_text = '; '.join(claimed) if claimed else 'none'
        print('%s[%s] %s claimed resources: %s' % (c.name, c.type, c.state, claimed_text))


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
    req.strictness = _strict_value()

    if hasattr(req, 'start_asap'):
        req.start_asap = True
    if hasattr(req, 'activate_asap'):
        req.activate_asap = True
    if hasattr(req, 'timeout'):
        try:
            from builtin_interfaces.msg import Duration
            req.timeout = Duration(sec=5, nanosec=0)
        except Exception:
            pass

    resp = _call_service('controller_manager/switch_controller', SwitchController, req)

    if resp.ok:
        if start_controllers:
            print("Started controllers: " + ", ".join(start_controllers))
        if stop_controllers:
            print("Stopped controllers: " + ", ".join(stop_controllers))
        return True
    else:
        print("Error when switching controllers")
        return False