import rospy

class BehaviorEngineVariantV3(object):
    def __init__(self, behavior):
        self._behavior = behavior
        self._log = rospy.loginfo

    def _process_state_feedback(self, state, feedback):
        # TODO
        # The LLM must:
        #   1) examine feedback (e.g., feedback.success or feedback.progress)
        #   2) return or set appropriate signals to the engine (e.g., call event posting or set state outcome)
        #   3) be robust to missing fields (use getattr)
        # END

