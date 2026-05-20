#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ListControllerTypes,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController,
    ReloadControllerLibraries
)

def _call_service(service_name, service_type, request):
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('controller_manager_interface_node')
    client = node.create_client(service_type, service_name)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for service {service_name}...')
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.destroy_node()
    return response

def list_controller_types():
    req = ListControllerTypes.Request()
    resp = _call_service('controller_manager/list_controller_types', ListControllerTypes, req)
    for t in resp.types:
        print(t)

def reload_libraries(force_kill, restore=False):
    req = ReloadControllerLibraries.Request()
    req.force_kill = force_kill
    resp = _call_service('controller_manager/reload_controller_libraries', ReloadControllerLibraries, req)
    return resp.ok

def list_controllers():
    req = ListControllers.Request()
    resp = _call_service('controller_manager/list_controllers', ListControllers, req)
    for c in resp.controller:
        interfaces = ', '.join(c.claimed_interfaces)
        print(f"{c.name} - {c.type} (state: {c.state})")
        print(f"  Claimed interfaces: {interfaces}")

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
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.STRICT
    resp = _call_service('controller_manager/switch_controller', SwitchController, req)
    return resp.ok