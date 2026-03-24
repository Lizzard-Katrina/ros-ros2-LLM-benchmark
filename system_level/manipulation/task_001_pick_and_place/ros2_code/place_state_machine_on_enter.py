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
import os
import ament_index_python.packages
from statemachine import State, StateMachine
from statemachine.contrib.diagram import DotGraphMachine

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
    
    def __init__(self, controller, node):
        self.controller = controller
        self.node = node
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        self.node.get_logger().info('\n' + 80*'=')
        self.node.get_logger().info("*** Pick-and-Place Mission Begins! ***") 
        self.node.get_logger().info(80*'=')

        super().__init__()

    def are_objects_detected(self) -> bool:
        """Guard to transition to 'selecting_object' state when in 'home' state."""
        return self.controller.are_objects_on_workbench()

    # Actions that occur when entering states
    def on_enter_home(self) -> None:
        """Moves robot to home position and triggers the 'select_object' event."""
        self.node.get_logger().info("Moving to home position..") 
        self.controller.panda.move_to_neutral()
        self.object_selected = False
        self.object_placed = False
        self.node.get_clock().sleep_for(seconds=0.2)

        self.send("select_object")


    def on_enter_selecting_object(self) -> None:
        """Selects an object to pick from the workbench and triggers the 'pick_object' event."""
        objects = self.controller.get_workbench_objects()
        if objects:
            import random
            self.currently_selected_object = random.choice(objects)
            self.object_selected = True
            self.node.get_logger().info(f"Selected object: {self.currently_selected_object}")
        else:
            self.object_selected = False
            self.node.get_logger().warn("No objects detected during selection.")
        self.send("pick_object")

    def on_enter_picking_and_placing(self) -> None:
        """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
        if self.currently_selected_object:
            success = self.controller.pick_and_place(self.currently_selected_object)
            if success:
                self.object_placed = True
                self.node.get_logger().info(f"Successfully placed object: {self.currently_selected_object}")
            else:
                self.object_placed = False
                self.node.get_logger().error(f"Failed to place object: {self.currently_selected_object}")
        else:
            self.object_placed = False
            self.node.get_logger().error("No object was selected to pick and place.")
        self.send("get_ready")
    
    def on_enter_done(self) -> None:
        """Reports that the robot has finished its pick-and-place tasks."""
        self.node.get_logger().info('\n' + 60*'=')
        self.node.get_logger().info("*** Mission Complete! *** \nAll objects have been placed in their bins.") 
        self.node.get_logger().info(60*'=')
        rclpy.shutdown()
        exit()


def create_state_machine_graph():
    """Creates and saves an image of the state machine graph."""
    graph = DotGraphMachine(PickAndPlaceStateMachine)
    dot = graph()
    pkg_path = ament_index_python.packages.get_package_share_directory('pick_and_place')
    file_path = os.path.join(pkg_path, 'images')
    if not os.path.exists(file_path):
        os.makedirs(file_path)
    dot.write_png(os.path.join(file_path, "state_machine.png"))
    print("\n State machine image saved! \n")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_and_place_state_machine")

    # create_state_machine_graph()
    controller = Controller()
    state_machine = PickAndPlaceStateMachine(controller, node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":     
    main()
