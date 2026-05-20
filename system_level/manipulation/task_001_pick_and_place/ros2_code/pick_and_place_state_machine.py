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
import os
from ament_index_python.packages import get_package_share_directory
from statemachine import State, StateMachine
from statemachine.contrib.diagram import DotGraphMachine
import time

from controller import Controller


class PickAndPlaceStateMachine(StateMachine):

    # States
    home = State("Home", initial=True)
    selecting_object = State("SelectingObject")                       
    picking_and_placing = State("PickingAndPlacing")                         
    done = State("Done", final=True)

    # Events & Transitions
    select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
    pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
    get_ready = picking_and_placing.to(home, cond="object_placed")
    
    def __init__(self, controller):
        self.controller = controller
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        print('\n' + 80*'=')
        self.controller.get_logger().info("*** Pick-and-Place Mission Begins! ***") 
        print(80*'=')

        super().__init__()

    def are_objects_detected(self) -> bool:
        """Guard to transition to 'selecting_object' state when in 'home' state."""
        return self.controller.are_objects_on_workbench()

    # Actions that occur when entering states
    def on_enter_home(self) -> None:
        """Moves robot to home position and triggers the 'select_object' event."""
        self.controller.get_logger().info("Moving to home position..") 
        self.controller.panda.move_to_neutral()
        self.object_selected = False
        self.object_placed = False
        time.sleep(0.2)

        self.send("select_object")

    def on_enter_selecting_object(self) -> None:
        self.controller.get_logger().info("Selecting an object from the workbench..")
        self.currently_selected_object = self.controller.select_object()
        self.object_selected = True
        self.controller.get_logger().info(f"Selected object: {self.currently_selected_object}")
        self.send("pick_object")

    def on_enter_picking_and_placing(self) -> None:
        """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
        self.controller.get_logger().info("Starting pick & place operation of selected object..") 
        self.controller.move_object(self.currently_selected_object)
        self.object_placed = True
        self.controller.get_logger().info("Object placed in its bin.")

        self.send("get_ready")
    
    def on_enter_done(self) -> None:
        """Reports that the robot has finished its pick-and-place tasks."""
        print('\n' + 60*'=')
        self.controller.get_logger().info("*** Mission Complete! *** \nAll objects have been placed in their bins.") 
        print(60*'=')
        exit()


def create_state_machine_graph():
    """Creates and saves an image of the state machine graph."""
    graph = DotGraphMachine(PickAndPlaceStateMachine)
    dot = graph()
    file_path = os.path.join(get_package_share_directory('pick_and_place'), 'images/')
    dot.write_png(file_path + "state_machine.png")
    print("\n State machine image saved! \n")


if __name__ == "__main__":     
    rclpy.init()
    controller = Controller()
    sm = PickAndPlaceStateMachine(controller)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()