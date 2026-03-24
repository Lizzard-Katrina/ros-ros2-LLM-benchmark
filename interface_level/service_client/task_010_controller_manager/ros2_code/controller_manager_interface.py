Here is the converted ROS2 code:
```python
#! /usr/bin/env python
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import *


class ControllerManagerClient(Node):
    def __init__(self):
        super().__init__('controller_manager_client')
        self.cli = self.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
        self.cli_load = self.create_client(LoadController, 'controller_manager/load_controller')
        self.cli_unload = self.create_client(UnloadController, 'controller_manager/unload_controller')
        self.cli_switch = self.create_client(SwitchController, 'controller_manager/switch_controller')

    def list_controller_types(self):
        self.cli.wait_for_service()
        req = ListControllerTypes.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for t in resp.types:
            self.get_logger().info(t)

    def reload_libraries(self, force_kill, restore=False):
        # TODO: Create the required service clients
        # Perform the reload service call and return a boolean that reflects the service response.
        # If restore is enabled, snapshot controllers before reload and restore their state.
        # END OF TODO
        pass

    def list_controllers(self):
        # TODO: 
        # Call the list_controllers service and print a human-readable summary.
        # For each, aggregate claimed hardware interfaces and format output.
        # END OF TODO
        pass

    def load_controller(self, name):
        self.cli_load.wait_for_service()
        req = LoadController.Request()
        req.name = name
        future = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            self.get_logger().info("Loaded '{}'".format(name))
            return True
        else:
            self.get_logger().error("Error when loading '{}'".format(name))
            return False

    def unload_controller(self, name):
        self.cli_unload.wait_for_service()
        req = UnloadController.Request()
        req.name = name
        future = self.cli_unload.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            self.get_logger().info("Unloaded '{}' successfully".format(name))
            return True
        else:
            self.get_logger().error("Error when unloading '{}'".format(name))
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
        # TODO: Call the switch_controller service with proper request fields
        # Return a boolean
        # END OF TODO
        self.cli_switch.wait_for_service()
        req = SwitchController.Request()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.ok


def main(args=None):
    rclpy.init(args=args)
    controller_manager_client = ControllerManagerClient()
    try:
        # Call functions here
        controller_manager_client.list_controller_types()
    finally:
        controller_manager_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()