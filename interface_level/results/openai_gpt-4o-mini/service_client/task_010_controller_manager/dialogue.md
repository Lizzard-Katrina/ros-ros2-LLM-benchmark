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
#! /usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes, LoadController, UnloadController, SwitchController
from controller_manager_msgs.srv import ListControllerTypesRequest, LoadControllerRequest, UnloadControllerRequest, SwitchControllerRequest


class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager')

    def list_controller_types(self):
        self.get_logger().info('Waiting for controller_manager/list_controller_types service...')
        self.create_service(ListControllerTypes, 'controller_manager/list_controller_types', self.list_controller_types_callback)
        s = self.create_client(ListControllerTypes, 'controller_manager/list_controller_types')
        s.wait_for_service()
        req = ListControllerTypesRequest()
        future = s.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for t in resp.types:
            self.get_logger().info(t)

    def reload_libraries(self, force_kill, restore=False):
        # TODO: Create the required service clients
        s_load = self.create_client(LoadController, 'controller_manager/load_controller')
        s_unload = self.create_client(UnloadController, 'controller_manager/unload_controller')
        # Perform the reload service call and return a boolean that reflects the service response.
        # If restore is enabled, snapshot controllers before reload and restore their state.
        # END OF TODO

    def list_controllers(self):
        # TODO: 
        s = self.create_client(ListControllers, 'controller_manager/list_controllers')
        s.wait_for_service()
        req = ListControllersRequest()
        future = s.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        for controller in resp.controller:
            self.get_logger().info(f"Controller: {controller.name}, State: {controller.state}, Hardware Interfaces: {controller.hardware_interface}")

        # END OF TODO

    def load_controller(self, name):
        self.get_logger().info(f'Waiting for controller_manager/load_controller service...')
        s = self.create_client(LoadController, 'controller_manager/load_controller')
        s.wait_for_service()
        req = LoadControllerRequest(name=name)
        future = s.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            self.get_logger().info(f"Loaded '{name}'")
            return True
        else:
            self.get_logger().error(f"Error when loading '{name}'")
            return False

    def unload_controller(self, name):
        self.get_logger().info(f'Waiting for controller_manager/unload_controller service...')
        s = self.create_client(UnloadController, 'controller_manager/unload_controller')
        s.wait_for_service()
        req = UnloadControllerRequest(name=name)
        future = s.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.ok:
            self.get_logger().info(f"Unloaded '{name}' successfully")
            return True
        else:
            self.get_logger().error(f"Error when unloading '{name}'")
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
        s = self.create_client(SwitchController, 'controller_manager/switch_controller')
        s.wait_for_service()
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = 1  # or appropriate value
        future = s.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.ok
        # END OF TODO


def main(args=None):
    rclpy.init(args=args)
    controller_manager = ControllerManager()
    rclpy.spin(controller_manager)
    controller_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```