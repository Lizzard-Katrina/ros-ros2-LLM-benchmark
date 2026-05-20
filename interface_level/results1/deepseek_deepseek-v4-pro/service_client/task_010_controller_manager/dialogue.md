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
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, ListControllers, LoadController, UnloadController, SwitchController
from controller_manager_msgs.srv import ListControllerTypes as ListControllerTypesSrv
from controller_manager_msgs.srv import ListControllers as ListControllersSrv
from controller_manager_msgs.srv import LoadController as LoadControllerSrv
from controller_manager_msgs.srv import UnloadController as UnloadControllerSrv
from controller_manager_msgs.srv import SwitchController as SwitchControllerSrv
from controller_manager_msgs.msg import ControllerState
import sys


class ControllerManagerInterface(Node):
    def __init__(self):
        super().__init__('controller_manager_interface')
        self.cli_list_types = self.create_client(ListControllerTypesSrv, 'controller_manager/list_controller_types')
        self.cli_list_controllers = self.create_client(ListControllersSrv, 'controller_manager/list_controllers')
        self.cli_load = self.create_client(LoadControllerSrv, 'controller_manager/load_controller')
        self.cli_unload = self.create_client(UnloadControllerSrv, 'controller_manager/unload_controller')
        self.cli_switch = self.create_client(SwitchControllerSrv, 'controller_manager/switch_controller')
        self.cli_reload = self.create_client(SwitchControllerSrv, 'controller_manager/reload_controller_libraries')

    def list_controller_types(self):
        while not self.cli_list_types.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        req = ListControllerTypesSrv.Request()
        future = self.cli_list_types.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for t in resp.types:
            print(t)

    def reload_libraries(self, force_kill, restore=False):
        if restore:
            # Snapshot current controllers
            while not self.cli_list_controllers.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('list_controllers service not available, waiting...')
            list_req = ListControllersSrv.Request()
            future_list = self.cli_list_controllers.call_async(list_req)
            rclpy.spin_until_future_complete(self, future_list)
            list_resp = future_list.result()
            active_controllers = [c.name for c in list_resp.controller if c.state == 'active']
            inactive_controllers = [c.name for c in list_resp.controller if c.state == 'inactive']
        else:
            active_controllers = []
            inactive_controllers = []

        while not self.cli_reload.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reload service not available, waiting...')
        reload_req = SwitchControllerSrv.Request()
        reload_req.activate_controllers = []
        reload_req.deactivate_controllers = []
        reload_req.strictness = SwitchControllerSrv.Request.BEST_EFFORT
        future_reload = self.cli_reload.call_async(reload_req)
        rclpy.spin_until_future_complete(self, future_reload)
        reload_resp = future_reload.result()

        if restore and active_controllers:
            # Restore previously active controllers
            switch_req = SwitchControllerSrv.Request()
            switch_req.activate_controllers = active_controllers
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchControllerSrv.Request.BEST_EFFORT
            future_switch = self.cli_switch.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future_switch)
            switch_resp = future_switch.result()
            return switch_resp.ok
        return reload_resp.ok

    def list_controllers(self):
        while not self.cli_list_controllers.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        req = ListControllersSrv.Request()
        future = self.cli_list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for c in resp.controller:
            hw_ifaces = []
            for claim in c.claimed_interfaces:
                hw_ifaces.append(claim)
            print(f"Controller: {c.name}, State: {c.state}, Type: {c.type}, HW Interfaces: {', '.join(hw_ifaces)}")

    def load_controller(self, name):
        while not self.cli_load.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        req = LoadControllerSrv.Request()
        req.name = name
        future = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            print(f"Loaded '{name}'")
            return True
        else:
            print(f"Error when loading '{name}'")
            return False

    def unload_controller(self, name):
        while not self.cli_unload.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        req = UnloadControllerSrv.Request()
        req.name = name
        future = self.cli_unload.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            print(f"Unloaded '{name}' successfully")
            return True
        else:
            print(f"Error when unloading '{name}'")
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
        while not self.cli_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('switch service not available, waiting...')
        req = SwitchControllerSrv.Request()
        req.activate_controllers = start_controllers
        req.deactivate_controllers = stop_controllers
        req.strictness = SwitchControllerSrv.Request.BEST_EFFORT
        future = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.ok


def main():
    rclpy.init(args=sys.argv)
    node = ControllerManagerInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()