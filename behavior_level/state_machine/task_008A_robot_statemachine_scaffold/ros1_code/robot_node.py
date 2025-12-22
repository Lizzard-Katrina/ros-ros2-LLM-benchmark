# variant_008A_robot_node_missing.py
import rospy
from rsm_core.state_machine import StateMachine

class RobotNode(object):
    """
    Node wrapper around the StateMachine.
    """
    def __init__(self):
        rospy.init_node("robot_state_machine_node")
        self.sm = StateMachine()
        self._setup_states()
        self._setup_transitions()

    def _setup_states(self):
        # TODO:
        #   Instantiate state objects:
        # END TODO

    def _setup_transitions(self):
        # TODO:
        #   Add Transition(...) objects linking states
        # END TODO

    def run(self):
        # TODO: set start state, then run
        # END TODO
if __name__ == "__main__":
    node = RobotNode()
    node.run()
