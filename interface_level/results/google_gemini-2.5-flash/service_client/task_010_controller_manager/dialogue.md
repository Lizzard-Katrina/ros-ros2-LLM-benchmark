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
#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, LoadController, UnloadController, SwitchController, ReloadControllerLibraries, ListControllers
from controller_manager_msgs.msg import ControllerState


class ControllerManagerClient(Node):
    def __init__(self):
        super().__init__('controller_manager_client')
        self.list_controller_types_cli = self.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
        self.load_controller_cli = self.create_client(LoadController, 'controller_manager/load_controller')
        self.unload_controller_cli = self.create_client(UnloadController, 'controller_manager/unload_controller')
        self.switch_controller_cli = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.reload_controller_libraries_cli = self.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
        self.list_controllers_cli = self.create_client(ListControllers, 'controller_manager/list_controllers')

    def list_controller_types(self):
        while not self.list_controller_types_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = ListControllerTypes.Request()
        future = self.list_controller_types_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for t in resp.types:
            print(t)

    def reload_libraries(self, force_kill, restore=False):
        while not self.reload_controller_libraries_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = ReloadControllerLibraries.Request()
        request.force_kill = force_kill
        request.restore_controllers = restore
        future = self.reload_controller_libraries_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.ok

    def list_controllers(self):
        while not self.list_controllers_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = ListControllers.Request()
        future = self.list_controllers_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for c in resp.controller:
            print(f"Controller: {c.name}")
            print(f"  Type: {c.type}")
            print(f"  State: {c.state}")
            print(f"  Claimed interfaces:")
            for interface in c.claimed_interfaces:
                print(f"    - {interface}")
            print("-" * 20)

    def load_controller(self, name):
        while not self.load_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = LoadController.Request()
        request.name = name
        future = self.load_controller_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            print("Loaded \'" + name + "\'")
            return True
        else:
            print("Error when loading \'" + name + "\'")
            return False

    def unload_controller(self, name):
        while not self.unload_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = UnloadController.Request()
        request.name = name
        future = self.unload_controller_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            print("Unloaded \'" + name + "\' successfully")
            return True
        else:
            print("Error when unloading \'" + name + "\'")
            return False

    def start_controller(self, name):
        return self.start_stop_controllers(start_controllers=[name])

    def start_controllers(self, names):
        return self.start_stop_controllers(start_controllers=names)

    def stop_controller(self, name):
        return self.start_stop_controllers(stop_controllers=[name])

    def stop_controllers(self, names):
        return self.start_stop_controllers(stop_controllers=names)

    def start_stop_controllers(self, start_controllers=[], stop_controllers=[]):
        while not self.switch_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SwitchController.Request()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = SwitchController.Request.STRICT
        request.activate_as_default = False
        request.timeout = rclpy.duration.Duration(seconds=0).to_msg()
        future = self.switch_controller_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.ok

```