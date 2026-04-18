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
from controller_manager_msgs.srv import *
import sys


def list_controller_types():
    rclpy.init()
    node = Node('list_controller_types_client')
    
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = ListControllerTypes.Request()
    future = client.call_async(req)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        resp = future.result()
        for t in resp.types:
            print(t)
    else:
        node.get_logger().error('Service call failed')
    
    node.destroy_node()
    rclpy.shutdown()


def reload_libraries(force_kill, restore=False):
    rclpy.init()
    node = Node('reload_libraries_client')
    
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    switch_client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    for client in [reload_client, list_client, switch_client]:
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'Service {client.srv_name} not available, waiting...')
    
    if restore:
        list_req = ListControllers.Request()
        list_future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, list_future)
        
        if list_future.result() is None:
            node.get_logger().error('Failed to get controller list for restore')
            node.destroy_node()
            rclpy.shutdown()
            return False
        
        controllers = list_future.result().controller
        
        for controller in controllers:
            if controller.state == 'running':
                switch_req = SwitchController.Request()
                switch_req.stop_controllers = [controller.name]
                switch_req.strictness = SwitchController.Request.BEST_EFFORT
                switch_future = switch_client.call_async(switch_req)
                rclpy.spin_until_future_complete(node, switch_future)
    
    reload_req = ReloadControllerLibraries.Request()
    reload_req.force_kill = force_kill
    reload_future = reload_client.call_async(reload_req)
    rclpy.spin_until_future_complete(node, reload_future)
    
    if reload_future.result() is None:
        node.get_logger().error('Reload service call failed')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    result = reload_future.result().ok
    
    if restore and result:
        for controller in controllers:
            if controller.state == 'running':
                switch_req = SwitchController.Request()
                switch_req.start_controllers = [controller.name]
                switch_req.strictness = SwitchController.Request.BEST_EFFORT
                switch_future = switch_client.call_async(switch_req)
                rclpy.spin_until_future_complete(node, switch_future)
    
    node.destroy_node()
    rclpy.shutdown()
    return result


def list_controllers():
    rclpy.init()
    node = Node('list_controllers_client')
    
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = ListControllers.Request()
    future = client.call_async(req)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        resp = future.result()
        for controller in resp.controller:
            print(f"\nController: {controller.name}")
            print(f"  Type: {controller.type}")
            print(f"  State: {controller.state}")
            print(f"  Hardware Interfaces: {', '.join(controller.claimed_interfaces)}")
            print(f"  Resources: {', '.join(controller.resources)}")
    else:
        node.get_logger().error('Service call failed')
    
    node.destroy_node()
    rclpy.shutdown()


def load_controller(name):
    rclpy.init()
    node = Node('load_controller_client')
    
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = LoadController.Request()
    req.name = name
    future = client.call_async(req)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        resp = future.result()
        if resp.ok:
            print(f"Loaded '{name}'")
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            print(f"Error when loading '{name}'")
            node.destroy_node()
            rclpy.shutdown()
            return False
    else:
        node.get_logger().error('Service call failed')
        node.destroy_node()
        rclpy.shutdown()
        return False


def unload_controller(name):
    rclpy.init()
    node = Node('unload_controller_client')
    
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = UnloadController.Request()
    req.name = name
    future = client.call_async(req)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        resp = future.result()
        if resp.ok:
            print(f"Unloaded '{name}' successfully")
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            print(f"Error when unloading '{name}'")
            node.destroy_node()
            rclpy.shutdown()
            return False
    else:
        node.get_logger().error('Service call failed')
        node.destroy_node()
        rclpy.shutdown()
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
    rclpy.init()
    node = Node('switch_controller_client')
    
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.BEST_EFFORT
    req.start_asap = False
    req.timeout = 5.0
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        resp = future.result()
        node.destroy_node()
        rclpy.shutdown()
        return resp.ok
    else:
        node.get_logger().error('Service call failed')
        node.destroy_node()
        rclpy.shutdown()
        return False
```