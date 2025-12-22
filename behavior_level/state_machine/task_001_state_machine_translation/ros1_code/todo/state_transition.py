class TurnstileSM:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['DONE'])
        with self.sm:
            # --------------------------
            # TODO: Translate state addition and transition mapping to ROS2-compatible SMACH
            # END

    def run(self):
        self.sm.execute()
