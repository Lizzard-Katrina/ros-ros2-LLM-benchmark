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
from controller_manager_msgs.srv import ListControllerTypes, ListControllers, LoadController, UnloadController, SwitchController, ReloadControllerLibraries


class ControllerManagerClient(Node):
    def __init__(self):
        super().__init__('controller_manager_client')


def list_controller_types():
    rclpy.init()
    node = ControllerManagerClient()
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting...')
    
    request = ListControllerTypes.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    for t, b in zip(response.types, response.base_classes):
        print(f"{t} - {b}")
    
    node.destroy_node()
    rclpy.shutdown()


def reload_libraries(force_kill, restore=False):
    rclpy.init()
    node = ControllerManagerClient()
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    
    while not reload_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('reload service not available, waiting...')
    
    if restore:
        list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
        switch_client = node.create_client(SwitchController, 'controller_manager/switch_controller')
        
        while not list_client.wait_for_service(timeout_sec=1.0) or not switch_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('services not available, waiting...')
        
        # Get current controllers state
        list_request = ListControllers.Request()
        list_future = list_client.call_async(list_request)
        rclpy.spin_until_future_complete(node, list_future)
        controllers = list_future.result().controller
        
        running = [c.name for c in controllers if c.state == 'active']
    
    # Call reload service
    reload_request = ReloadControllerLibraries.Request()
    reload_request.force_kill = force_kill
    reload_future = reload_client.call_async(reload_request)
    rclpy.spin_until_future_complete(node, reload_future)
    result = reload_future.result().ok
    
    # Restore controllers if requested
    if restore and result and running:
        switch_request = SwitchController.Request()
        switch_request.start_controllers = running
        switch_request.strictness = SwitchController.Request.BEST_EFFORT
        switch_future = switch_client.call_async(switch_request)
        rclpy.spin_until_future_complete(node, switch_future)
    
    node.destroy_node()
    rclpy.shutdown()
    return result


def list_controllers():
    rclpy.init()
    node = ControllerManagerClient()
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting...')
    
    request = ListControllers.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    if not response.controller:
        print("No controllers are loaded in controller manager")
    else:
        print("Loaded controllers:")
        for controller in response.controller:
            print(f"  {controller.name} - {controller.state}")
            
            if controller.claimed_interfaces:
                print("    Claimed interfaces:")
                for interface in controller.claimed_interfaces:
                    print(f"      {interface}")
    
    node.destroy_node()
    rclpy.shutdown()


def load_controller(name):
    rclpy.init()
    node = ControllerManagerClient()
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting...')
    
    request = LoadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    node.destroy_node()
    rclpy.shutdown()
    
    if response.ok:
        print(f"Loaded '{name}'")
        return True
    else:
        print(f"Error when loading '{name}'")
        return False


def unload_controller(name):
    rclpy.init()
    node = ControllerManagerClient()
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting...')
    
    request = UnloadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    node.destroy_node()
    rclpy.shutdown()
    
    if response.ok:
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
    rclpy.init()
    node = ControllerManagerClient()
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting...')
    
    request = SwitchController.Request()
    request.start_controllers = start_controllers
    request.stop_controllers = stop_controllers
    request.strictness = SwitchController.Request.BEST_EFFORT
    request.start_asap = False
    request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    node.destroy_node()
    rclpy.shutdown()
    
    return response.ok
```