#!/usr/bin/env python3
from task_001_smach_basic.smach_minimal import State

__all__ = ['RosState']


class RosState(State):
    """
    A state that can interact with a ROS node.
    """
    def __init__(self, node, **kwargs):
        State.__init__(self, **kwargs)
        self.__node = node

    @property
    def node(self):
        return self.__node