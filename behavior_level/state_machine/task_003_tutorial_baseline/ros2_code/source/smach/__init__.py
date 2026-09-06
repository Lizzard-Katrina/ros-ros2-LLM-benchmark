"""
Minimal smach compatibility shim for ROS2.
Provides the State base class with the interface expected by code
migrated from ROS1 smach.
"""


class State:
    """Minimal smach.State replacement for ROS2."""

    def __init__(self, outcomes=None, input_keys=None, output_keys=None,
                 io_keys=None):
        self._outcomes = outcomes or []
        self._input_keys = input_keys or []
        self._output_keys = output_keys or []
        self._io_keys = io_keys or []
        self._preempt_requested = False

    def execute(self, ud):
        raise NotImplementedError("execute() must be overridden")

    def preempt_requested(self):
        return self._preempt_requested

    def service_preempt(self):
        self._preempt_requested = False

    def request_preempt(self):
        self._preempt_requested = True