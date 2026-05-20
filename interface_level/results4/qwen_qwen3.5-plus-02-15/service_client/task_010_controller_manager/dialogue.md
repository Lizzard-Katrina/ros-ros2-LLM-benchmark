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
#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, ReloadControllerLibraries, ListControllers, LoadController, UnloadController, SwitchController


def list_controller_types():
    node = Node('controller_manager_interface_temp')
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    req = ListControllerTypes.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp:
        for t in resp.types:
            print(t)
    node.destroy_node()


def reload_libraries(force_kill, restore=False):
    node = Node('controller_manager_interface_temp')
    
    # Create the required service clients
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    load_client = node.create_client(LoadController, 'controller_manager/load_controller')
    start_client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    clients = [reload_client, list_client, load_client, start_client]
    for c in clients:
        while not c.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

    # If restore is enabled, snapshot controllers before reload and restore their state.
    running_controllers = []
    if restore:
        req_list = ListControllers.Request()
        future_list = list_client.call_async(req_list)
        rclpy.spin_until_future_complete(node, future_list)
        resp_list = future_list.result()
        
        if resp_list:
            for c in resp_list.controller:
                if c.state == 'running':
                    running_controllers.append(c.name)

    # Perform the reload service call
    req_reload = ReloadControllerLibraries.Request()
    req_reload.force_kill = force_kill
    future_reload = reload_client.call_async(req_reload)
    rclpy.spin_until_future_complete(node, future_reload)
    resp_reload = future_reload.result()
    
    result = False
    if resp_reload:
        result = resp_reload.ok
        
        # Restore state if restore was enabled and reload succeeded
        if restore and result:
            for name in running_controllers:
                # Load
                req_load = LoadController.Request()
                req_load.name = name
                future_load = load_client.call_async(req_load)
                rclpy.spin_until_future_complete(node, future_load)
                
                # Start
                req_switch = SwitchController.Request()
                req_switch.start_controllers = [name]
                req_switch.strictness = SwitchController.Request.BEST_EFFORT
                future_switch = start_client.call_async(req_switch)
                rclpy.spin_until_future_complete(node, future_switch)

    node.destroy_node()
    return result


def list_controllers():
    node = Node('controller_manager_interface_temp')
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    
    req = ListControllers.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    
    if resp:
        print("List of controllers:")
        for c in resp.controller:
            claimed_interfaces = ", ".join(c.claimed_interfaces)
            print(f"- {c.name} [{c.state}] (Hardware: {claimed_interfaces})")
    
    node.destroy_node()


def load_controller(name):
    node = Node('controller_manager_interface_temp')
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
        
    req = LoadController.Request()
    req.name = name
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    
    if resp and resp.ok:
        print("Loaded '" + name + "'")
        node.destroy_node()
        return True
    else:
        print("Error when loading '" + name + "'")
        node.destroy_node()
        return False


def unload_controller(name):
    node = Node('controller_manager_interface_temp')
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
        
    req = UnloadController.Request()
    req.name = name
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    
    if resp and resp.ok:
        print("Unloaded '" + name + "' successfully")
        node.destroy_node()
        return True
    else:
        print("Error when unloading '" + name + "'")
        node.destroy_node()
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
    node = Node('controller_manager_interface_temp')
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.BEST_EFFORT
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    
    node.destroy_node()
    
    if resp:
        return resp.ok
    return False