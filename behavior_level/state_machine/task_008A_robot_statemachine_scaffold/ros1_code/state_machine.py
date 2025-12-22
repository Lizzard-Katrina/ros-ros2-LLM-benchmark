# variant_008A_state_machine_missing.py
import rospy
from rsm_core.state import State
from rsm_core.transitions import Transition

class StateMachine(object):

    def __init__(self):
        self._states = {}            # name → State instance
        self._transitions = {}       # name → [Transition]
        self._active_state = None
        self._running = False

    def add_state(self, name, state_obj):
        self._states[name] = state_obj

    def add_transition(self, from_state, transition_obj):
        self._transitions.setdefault(from_state, []).append(transition_obj)

    def set_start(self, state_name):
        self._active_state = self._states[state_name]

    def run(self):
        self._running = True
        rospy.loginfo("[RSM] Starting state machine...")

        self._active_state.on_enter()

        while not rospy.is_shutdown() and self._running:
            outcome = self._active_state.execute()

            # TODO: Implement transition logic:
            #   - Look up transitions for current state
            #   - Evaluate each transition
            #   - If one fires:
            #       switch state
            #       3) call new_state.on_enter()
            # END TODO

            rospy.sleep(0.05)
