#!/usr/bin/env python3
"""
Mock controller_manager service servers for testing.
Provides all six services that controller_manager_interface.py calls.
"""
import sys
import rclpy
from rclpy.node import Node
from task_010_controller_manager.srv import (
    ListControllerTypes,
    ListControllers,
    LoadController,
    UnloadController,
    SwitchController,
    ReloadControllerLibraries,
)
from task_010_controller_manager.msg import ControllerState


class MockControllerManager(Node):
    def __init__(self):
        super().__init__('mock_controller_manager')

        self.create_service(
            ListControllerTypes,
            'controller_manager/list_controller_types',
            self.handle_list_controller_types,
        )
        self.create_service(
            ListControllers,
            'controller_manager/list_controllers',
            self.handle_list_controllers,
        )
        self.create_service(
            LoadController,
            'controller_manager/load_controller',
            self.handle_load_controller,
        )
        self.create_service(
            UnloadController,
            'controller_manager/unload_controller',
            self.handle_unload_controller,
        )
        self.create_service(
            SwitchController,
            'controller_manager/switch_controller',
            self.handle_switch_controller,
        )
        self.create_service(
            ReloadControllerLibraries,
            'controller_manager/reload_controller_libraries',
            self.handle_reload_libraries,
        )
        self.get_logger().info('Mock controller manager ready')

    def handle_list_controller_types(self, request, response):
        response.types = ['joint_state_controller/JointStateController',
                          'effort_controllers/JointPositionController']
        response.base_classes = ['controller_interface::ControllerInterface',
                                 'controller_interface::ControllerInterface']
        return response

    def handle_list_controllers(self, request, response):
        c1 = ControllerState()
        c1.name = 'joint_state_controller'
        c1.state = 'running'
        c1.type = 'joint_state_controller/JointStateController'
        c1.claimed_interfaces = ['hardware_interface::JointStateInterface']

        c2 = ControllerState()
        c2.name = 'arm_controller'
        c2.state = 'stopped'
        c2.type = 'effort_controllers/JointPositionController'
        c2.claimed_interfaces = ['hardware_interface::EffortJointInterface']

        response.controller = [c1, c2]
        return response

    def handle_load_controller(self, request, response):
        response.ok = True
        return response

    def handle_unload_controller(self, request, response):
        response.ok = True
        return response

    def handle_switch_controller(self, request, response):
        response.ok = True
        return response

    def handle_reload_libraries(self, request, response):
        response.ok = True
        return response


def main():
    rclpy.init()
    node = MockControllerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()