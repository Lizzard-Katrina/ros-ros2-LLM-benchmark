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
from controller_manager_msgs.srv import ListControllerTypesRequest, ListControllersRequest, LoadControllerRequest, UnloadControllerRequest, SwitchControllerRequest
from controller_manager_msgs.srv import ReloadControllerLibraries, ReloadControllerLibrariesRequest


class ControllerManagerInterface(Node):
    def __init__(self):
        super().__init__('controller_manager_interface')
        self.list_types_client = self.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
        self.list_client = self.create_client(ListControllers, 'controller_manager/list_controllers')
        self.load_client = self.create_client(LoadController, 'controller_manager/load_controller')
        self.unload_client = self.create_client(UnloadController, 'controller_manager/unload_controller')
        self.switch_client = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.reload_client = self.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')

    def list_controller_types(self):
        if not self.list_types_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service list_controller_types not available')
            return
        req = ListControllerTypesRequest()
        future = self.list_types_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            for t in future.result().types:
                print(t)
        else:
            self.get_logger().error('Failed to call list_controller_types')

    def reload_libraries(self, force_kill, restore=False):
        if not self.reload_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service reload_controller_libraries not available')
            return False

        if restore:
            # Snapshot current controllers state
            if not self.list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Service list_controllers not available for snapshot')
                return False
            list_req = ListControllersRequest()
            list_future = self.list_client.call_async(list_req)
            rclpy.spin_until_future_complete(self, list_future)
            if list_future.result() is None:
                self.get_logger().error('Failed to snapshot controllers')
                return False
            snapshot = list_future.result().controller

        req = ReloadControllerLibrariesRequest()
        req.force_kill = force_kill
        future = self.reload_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to call reload_controller_libraries')
            return False
        success = future.result().ok

        if restore and success:
            # Restore controllers from snapshot
            start_list = []
            stop_list = []
            for c in snapshot:
                if c.state == 'active':
                    start_list.append(c.name)
                elif c.state == 'inactive':
                    stop_list.append(c.name)
            if start_list or stop_list:
                self.start_stop_controllers(start_controllers=start_list, stop_controllers=stop_list)

        return success

    def list_controllers(self):
        if not self.list_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service list_controllers not available')
            return
        req = ListControllersRequest()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            resp = future.result()
            for c in resp.controller:
                hw_ifaces = []
                for claim in c.claimed_interfaces:
                    hw_ifaces.append(claim)
                print(f"Controller: {c.name}, State: {c.state}, Type: {c.type}, HW Interfaces: {hw_ifaces}")
        else:
            self.get_logger().error('Failed to call list_controllers')

    def load_controller(self, name):
        if not self.load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service load_controller not available')
            return False
        req = LoadControllerRequest(name=name)
        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            resp = future.result()
            if resp.ok:
                print(f"Loaded '{name}'")
                return True
            else:
                print(f"Error when loading '{name}'")
                return False
        else:
            self.get_logger().error(f'Failed to call load_controller for {name}')
            return False

    def unload_controller(self, name):
        if not self.unload_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service unload_controller not available')
            return False
        req = UnloadControllerRequest(name=name)
        future = self.unload_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            resp = future.result()
            if resp.ok:
                print(f"Unloaded '{name}' successfully")
                return True
            else:
                print(f"Error when unloading '{name}'")
                return False
        else:
            self.get_logger().error(f'Failed to call unload_controller for {name}')
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
        if not self.switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service switch_controller not available')
            return False
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchControllerRequest.BEST_EFFORT
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().ok
        else:
            self.get_logger().error('Failed to call switch_controller')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManagerInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()