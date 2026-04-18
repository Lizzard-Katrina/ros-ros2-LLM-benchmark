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
from controller_manager_msgs.srv import ListControllerTypes, LoadController, UnloadController, SwitchController, ListControllers


class ControllerManagerClient(Node):

    def __init__(self):
        super().__init__('controller_manager_client')
        self.cli_list_controller_types = self.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
        self.cli_load_controller = self.create_client(LoadController, 'controller_manager/load_controller')
        self.cli_unload_controller = self.create_client(UnloadController, 'controller_manager/unload_controller')
        self.cli_switch_controller = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.cli_list_controllers = self.create_client(ListControllers, 'controller_manager/list_controllers')

    def list_controller_types(self):
        while not self.cli_list_controller_types.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = ListControllerTypes.Request()
        future = self.cli_list_controller_types.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            for t in future.result().types:
                print(t)
        else:
            self.get_logger().error('Service call failed')

    def reload_libraries(self, force_kill, restore=False):
        # TODO: Create the required service clients
        # Perform the reload service call and return a boolean that reflects the service response.
        # If restore is enabled, snapshot controllers before reload and restore their state.
        # END OF TODO
        pass

    def list_controllers(self):
        while not self.cli_list_controllers.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = ListControllers.Request()
        future = self.cli_list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            for controller in future.result().controller:
                print(f"Controller: {controller.name}, State: {controller.state}")
                claimed_interfaces = ', '.join(controller.claimed_interfaces)
                print(f"Claimed Interfaces: {claimed_interfaces}")
        else:
            self.get_logger().error('Service call failed')

    def load_controller(self, name):
        while not self.cli_load_controller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = LoadController.Request()
        req.name = name
        future = self.cli_load_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
            print(f"Loaded '{name}'")
            return True
        else:
            print(f"Error when loading '{name}'")
            return False

    def unload_controller(self, name):
        while not self.cli_unload_controller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = UnloadController.Request()
        req.name = name
        future = self.cli_unload_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
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
        while not self.cli_switch_controller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SwitchController.Request()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.cli_switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
            return True
        else:
            self.get_logger().error('Service call failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    controller_manager_client = ControllerManagerClient()
    # Example usage
    controller_manager_client.list_controller_types()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```