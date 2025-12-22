# variant_008A_transitions_missing.py

class Transition(object):
    """
    Simplified + merged version of repo transition logic:
    TransitionCondition → Transition → evaluated in StateMachine
    """

    def __init__(self, outcome, next_state_name, condition=None):
        self.outcome = outcome
        self.next_state_name = next_state_name
        self.condition = condition  # function(state_machine) → bool

    def evaluate(self, state_machine):
        """
        Original: evaluates TransitionCondition list.
        """
        # TODO:
        # END TODO
