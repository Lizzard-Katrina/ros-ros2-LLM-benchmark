#!/usr/bin/env python3
"""
Minimal smach-like classes sufficient for ServiceState usage.
This replaces the ROS1 'smach' package which is not available in ROS2 Humble.
"""

__all__ = [
    'State', 'UserData', 'Remapper', 'InvalidStateError',
    'has_smach_interface',
]


class InvalidStateError(Exception):
    pass


class UserData:
    """Minimal UserData container that behaves like a dict-like object with attribute access."""
    def __init__(self):
        self._data = {}

    def __getattr__(self, name):
        if name.startswith('_'):
            return object.__getattribute__(self, name)
        try:
            return self._data[name]
        except KeyError:
            raise AttributeError("UserData has no attribute '%s'" % name)

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
        else:
            self._data[name] = value

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value

    def __contains__(self, key):
        return key in self._data

    def keys(self):
        return self._data.keys()


class Remapper:
    """Minimal Remapper that wraps UserData, providing access to registered keys."""
    def __init__(self, ud, input_keys, output_keys, remap=None):
        self._ud = ud
        self._input_keys = input_keys
        self._output_keys = output_keys
        self._remap = remap or []

    def __getattr__(self, name):
        if name.startswith('_'):
            return object.__getattribute__(self, name)
        return self._ud[name]

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
        else:
            self._ud[name] = value

    def __getitem__(self, key):
        return self._ud[key]

    def __setitem__(self, key, value):
        self._ud[key] = value

    def __contains__(self, key):
        return key in self._ud

    def keys(self):
        return self._ud.keys()


class State:
    """Minimal smach.State replacement."""
    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        self._outcomes = set(outcomes or [])
        self._input_keys = set(input_keys or [])
        self._output_keys = set(output_keys or [])
        self._preempt_requested = False

    def register_outcomes(self, outcomes):
        self._outcomes.update(outcomes)

    def register_input_keys(self, keys):
        self._input_keys.update(keys)

    def register_output_keys(self, keys):
        self._output_keys.update(keys)

    def get_registered_outcomes(self):
        return self._outcomes

    def get_registered_input_keys(self):
        return self._input_keys

    def get_registered_output_keys(self):
        return self._output_keys

    def preempt_requested(self):
        return self._preempt_requested

    def service_preempt(self):
        self._preempt_requested = False

    def request_preempt(self):
        self._preempt_requested = True

    def execute(self, ud):
        raise NotImplementedError("State.execute() must be overridden")


def has_smach_interface(obj):
    """Check if an object has the smach callback interface."""
    if obj is None:
        return False
    return (hasattr(obj, 'get_registered_input_keys') and
            hasattr(obj, 'get_registered_output_keys'))