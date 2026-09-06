#!/usr/bin/env python3

"""
A state machine for pick-and-place tasks.

The robot starts in the home position. If objects are detected on the workbench,
it randomly selects one. Once an object is selected, the robot picks and places it to
the same color bin. Then, returns to the home position. If no objects are
detected on the workbench, it stops.

Migrated from ROS 1 to ROS 2 Humble.

Author: Elena Oikonomou (original ROS1)
Date:   Fall 2023
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import random
import threading
from concurrent.futures import Future

from pick_and_place.msg import DetectedObjectsStamped, DetectedObject


class Controller(Node):
    """Minimal ROS 2 Controller node that subscribes to /object_detection."""

    def __init__(self):
        super().__init__('controller')

        self.objects_on_workbench = []
        self._cb_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            DetectedObjectsStamped,
            '/object_detection',
            self.update_objects_callback,
            10,
            callback_group=self._cb_group
        )
        self.get_logger().info('Controller node initialized.')

    def update_objects_callback(self, msg: DetectedObjectsStamped) -> None:
        """Updates the objects that are currently on top of the workbench."""
        self.objects_on_workbench = msg.detected_objects

    def select_random_object(self) -> DetectedObject:
        """Selects an object at random from the ones on the workbench."""
        return random.choice(self.objects_on_workbench)

    def are_objects_on_workbench(self) -> bool:
        """Checks whether there are any objects on top of the workbench."""
        return len(self.objects_on_workbench) > 0

    def move_to_neutral(self):
        """Simulates moving to neutral/home position."""
        self.get_logger().info('Moving to neutral position.')
        time.sleep(0.1)

    def move_object(self, obj: DetectedObject):
        """Simulates moving an object to its bin."""
        self.get_logger().info(f'Moving object (color={obj.color}) to bin.')
        time.sleep(0.1)

    def select_object_async(self):
        """Asynchronously selects an object, returning a Future."""
        future = Future()

        def _do_select():
            try:
                obj = self.select_random_object()
                future.set_result(obj)
            except Exception as e:
                future.set_exception(e)

        t = threading.Thread(target=_do_select, daemon=True)
        t.start()
        return future


class PickAndPlaceStateMachine(Node):
    """State machine node for pick-and-place tasks using ROS 2."""

    def __init__(self, controller: Controller):
        super().__init__('pick_and_place_state_machine')

        self.controller = controller
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        # States
        self._state = 'home'
        self._running = True

        self.get_logger().info('=' * 80)
        self.get_logger().info('*** Pick-and-Place Mission Begins! ***')
        self.get_logger().info('=' * 80)

        # Use a timer to drive the state machine
        self._cb_group = ReentrantCallbackGroup()
        self._timer = self.create_timer(0.5, self._tick, callback_group=self._cb_group)

    def _tick(self):
        """Main state machine tick driven by timer."""
        if not self._running:
            return

        if self._state == 'home':
            self.on_enter_home()
        elif self._state == 'selecting_object':
            self.on_enter_selecting_object()
        elif self._state == 'picking_and_placing':
            self.on_enter_picking_and_placing()
        elif self._state == 'done':
            self.on_enter_done()

    def are_objects_detected(self) -> bool:
        """Guard to transition to 'selecting_object' state when in 'home' state."""
        return self.controller.are_objects_on_workbench()

    def on_enter_home(self) -> None:
        """Moves robot to home position and triggers the 'select_object' event."""
        self.get_logger().info("Moving to home position..")
        self.controller.move_to_neutral()
        self.object_selected = False
        self.object_placed = False
        time.sleep(0.2)

        if self.are_objects_detected():
            self._state = 'selecting_object'
        else:
            self._state = 'done'

    def on_enter_selecting_object(self) -> None:
        """Selects an object using async Future pattern and triggers 'pick_object' event."""
        self.get_logger().info("Selecting object to pick..")

        # Use async Future pattern with add_done_callback for synchronization
        future = self.controller.select_object_async()

        def _on_selection_done(fut):
            try:
                self.currently_selected_object = fut.result()
                self.object_selected = True
                self.get_logger().info("Object selected.")
                self._state = 'picking_and_placing'
            except Exception as e:
                self.get_logger().error(f"Object selection failed: {e}")
                self._state = 'home'

        future.add_done_callback(_on_selection_done)

        # Wait for the future to complete with timeout
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if not future.done():
            self.get_logger().warn("Object selection timed out.")
            self._state = 'home'

    def on_enter_picking_and_placing(self) -> None:
        """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
        self.get_logger().info("Starting pick & place operation of selected object..")
        self.controller.move_object(self.currently_selected_object)
        self.object_placed = True
        self.get_logger().info("Object placed in its bin.")

        self._state = 'home'

    def on_enter_done(self) -> None:
        """Reports that the robot has finished its pick-and-place tasks."""
        self._running = False
        self.get_logger().info('=' * 60)
        self.get_logger().info("*** Mission Complete! ***")
        self.get_logger().info("All objects have been placed in their bins.")
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    pick_and_place_sm = PickAndPlaceStateMachine(controller)

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(pick_and_place_sm)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pick_and_place_sm.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()