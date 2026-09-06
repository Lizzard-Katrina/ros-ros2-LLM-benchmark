"""
Lightweight smach-like State base class for ROS2 migration.
Replaces the rospy-dependent smach library.
"""


class UserData:
    """Simple attribute-based userdata container."""
    pass


class State:
    """Minimal smach.State replacement."""

    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        self._outcomes = outcomes or []
        self._input_keys = input_keys or []
        self._output_keys = output_keys or []
        self._preempt_requested = False

    def execute(self, ud):
        raise NotImplementedError("Subclasses must implement execute()")

    def preempt_requested(self):
        return self._preempt_requested

    def request_preempt(self):
        self._preempt_requested = True

    def service_preempt(self):
        self._preempt_requested = False