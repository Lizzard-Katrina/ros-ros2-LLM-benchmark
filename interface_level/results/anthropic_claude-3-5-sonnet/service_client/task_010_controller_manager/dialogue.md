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
from controller_manager_msgs.srv import ListControllerTypes, ListControllers, LoadController, UnloadController, SwitchController
from controller_manager_msgs.msg import ControllerState

def list_controller_types():
    node = rclpy.create_node('controller_manager_client')
    client = node.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service not available')
        return
        
    request = ListControllerTypes.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        for t in future.result().types:
            print(t)
            
    node.destroy_node()

def reload_libraries(force_kill, restore=False):
    node = rclpy.create_node('controller_manager_client')
    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    switch_client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    if restore:
        # Get running controllers
        future = list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            running = [c.name for c in future.result().controller if c.state == 'running']
            
    # Call reload
    request = ReloadControllerLibraries.Request()
    request.force_kill = force_kill
    future = reload_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result() is not None and future.result().ok
    
    if restore and result:
        # Restore controllers
        request = SwitchController.Request()
        request.start_controllers = running
        request.strictness = SwitchController.Request.BEST_EFFORT
        future = switch_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
    node.destroy_node()
    return result

def list_controllers():
    node = rclpy.create_node('controller_manager_client')
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service not available')
        return
        
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        for c in future.result().controller:
            hwi = ' '.join(c.claimed_interfaces)
            print(f"'{c.name}' - {c.state} ({c.type})")
            if hwi:
                print(f"  Hardware interface: {hwi}")
                
    node.destroy_node()

def load_controller(name):
    node = rclpy.create_node('controller_manager_client')
    client = node.create_client(LoadController, 'controller_manager/load_controller')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service not available')
        return False
        
    request = LoadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None and future.result().ok:
        print(f"Loaded '{name}'")
        node.destroy_node()
        return True
    else:
        print(f"Error when loading '{name}'")
        node.destroy_node()
        return False

def unload_controller(name):
    node = rclpy.create_node('controller_manager_client')
    client = node.create_client(UnloadController, 'controller_manager/unload_controller')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service not available')
        return False
        
    request = UnloadController.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None and future.result().ok:
        print(f"Unloaded '{name}' successfully")
        node.destroy_node()
        return True
    else:
        print(f"Error when unloading '{name}'")
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
    node = rclpy.create_node('controller_manager_client')
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service not available')
        return False
        
    request = SwitchController.Request()
    request.start_controllers = start_controllers
    request.stop_controllers = stop_controllers
    request.strictness = SwitchController.Request.BEST_EFFORT
    request.start_asap = False
    request.timeout = 0.0
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result() is not None and future.result().ok
    node.destroy_node()
    return result
```