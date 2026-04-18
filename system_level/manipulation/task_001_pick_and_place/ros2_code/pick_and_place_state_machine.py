#!/usr/bin/env python3

"""
A state machine for pick-and-place tasks.

The robot starts in the home position. If objects are detected on the workbench,
it randomly selects one. Once an object is selected, the robot picks and places it to 
the same color bin. Then, returns to the home position. If no objects are 
detected on the workbench, it stops.

Author: Elena Oikonomou
Date:   Fall 2023
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pick_and_place.msg import DetectedObjectsStamped
from controller import Controller
from vision_object_detector import VisionObjectDetector

class PickAndPlaceStateMachine(Node):

    # States
    home = 'home'
    selecting_object = 'selecting_object'
    picking_and_placing = 'picking_and_placing'
    done = 'done'

    # Events & Transitions
    def __init__(self, controller, object_detector):
        super().__init__('pick_and_place_state_machine')
        self.controller = controller
        self.object_detector = object_detector
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None
        self.state = self.home

        self.detected_objects_sub = self.create_subscription(
            DetectedObjectsStamped,
            'detected_objects',
            self.detected_objects_callback,
            QoSProfile(depth=10))

        self.get_logger().info('*** Pick-and-Place Mission Begins! ***')

    def detected_objects_callback(self, msg: DetectedObjectsStamped) -> None:
        if self.state == self.home:
            if msg.detected_objects:
                self.state = self.selecting_object
                self.select_object()
            else:
                self.state = self.done
                self.done_state()
        elif self.state == self.selecting_object:
            self.select_object()
        elif self.state == self.picking_and_placing:
            self.pick_and_place()
        elif self.state == self.done:
            self.done_state()

    def select_object(self) -> None:
        # State transitions must be tethered to the actual completion of the controller's task.
        self.currently_selected_object = self.object_detector.blocks_on_workbench[0]
        self.state = self.picking_and_placing
        self.pick_and_place()

    def pick_and_place(self) -> None:
        self.controller.move_object(self.currently_selected_object)
        self.object_placed = True
        self.state = self.home
        self.get_logger().info('Object placed in its bin.')

    def done_state(self) -> None:
        self.get_logger().info('*** Mission Complete! *** \nAll objects have been placed in their bins.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    object_detector = VisionObjectDetector()
    controller = Controller()
    state_machine = PickAndPlaceStateMachine(controller, object_detector)
    executor = MultiThreadedExecutor()
    rclpy.spin(state_machine, executor=executor)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()