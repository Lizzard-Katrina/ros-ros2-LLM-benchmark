#!/usr/bin/env python3
"""
Runtime test for GetJointValuesState (task_006_flexbe_2).

This test exercises the actual translated get_joint_values_state.py by importing
the class and driving it with mocked FlexBE/proxy infrastructure, then asserting
on the concrete joint values it produces.
"""

import pytest
import sys
import os
import types
import threading
import time
from unittest.mock import MagicMock
from pathlib import Path


# ---------------------------------------------------------------------------
# We need to set up minimal mocks for flexbe_core and its proxy layer so we
# can import and unit-test GetJointValuesState without a running ROS graph.
# ---------------------------------------------------------------------------

def _setup_flexbe_mocks():
    """Create lightweight stand-ins for flexbe_core so the real file can import."""

    # ---- rclpy (real, but we only need minimal bits) ----
    import rclpy
    from rclpy.duration import Duration
    from rclpy.clock import Clock, ClockType

    # ---- sensor_msgs ----
    from sensor_msgs.msg import JointState

    # ---- flexbe_core.EventState mock ----
    class _MockEventState:
        """Minimal EventState stand-in."""
        def __init__(self, *args, **kwargs):
            outcomes = kwargs.get('outcomes', [])
            output_keys = kwargs.get('output_keys', [])
            input_keys = kwargs.get('input_keys', [])
            self._outcomes = outcomes
            self._output_keys = output_keys
            self._input_keys = input_keys
            # Provide a real ROS clock
            self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        def get_clock(self):
            return self._clock

    # ---- ProxySubscriberCached mock ----
    class _MockProxySubscriberCached:
        """Mock proxy that stores a buffer we can push messages into."""
        def __init__(self, topics_dict, inst_id=None):
            self._buffers = {}  # topic -> list of msgs
            self._buffer_enabled = {}

        def enable_buffer(self, topic):
            self._buffer_enabled[topic] = True
            if topic not in self._buffers:
                self._buffers[topic] = []

        def disable_buffer(self, topic):
            self._buffer_enabled[topic] = False

        def has_buffered(self, topic):
            return len(self._buffers.get(topic, [])) > 0

        def get_from_buffer(self, topic):
            buf = self._buffers.get(topic, [])
            if buf:
                return buf.pop(0)
            return None

        def push(self, topic, msg):
            """Test helper – push a message into the buffer."""
            if topic not in self._buffers:
                self._buffers[topic] = []
            self._buffers[topic].append(msg)

    # Build mock modules
    flexbe_core_mod = types.ModuleType('flexbe_core')
    flexbe_core_mod.EventState = _MockEventState
    flexbe_core_mod.Logger = MagicMock()

    proxy_mod = types.ModuleType('flexbe_core.proxy')
    proxy_mod.ProxySubscriberCached = _MockProxySubscriberCached

    sys.modules['flexbe_core'] = flexbe_core_mod
    sys.modules['flexbe_core.proxy'] = proxy_mod

    return _MockProxySubscriberCached


_MockProxy = _setup_flexbe_mocks()


# ---------------------------------------------------------------------------
# Now import the REAL translated file
# ---------------------------------------------------------------------------
# The file lives at the package root (get_joint_values_state.py)
_pkg_root = Path(__file__).resolve().parent
sys.path.insert(0, str(_pkg_root))

from get_joint_values_state import GetJointValuesState  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402


# ---------------------------------------------------------------------------
# Helper: simple userdata dict-like object
# ---------------------------------------------------------------------------
class _UserData(dict):
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        self[name] = value


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestGetJointValuesStateRuntime:
    """Runtime tests that actually instantiate and execute the translated state."""

    def test_basic_retrieval_all_joints_single_message(self):
        """All requested joints arrive in one message."""
        joints = ['joint_a', 'joint_b', 'joint_c']
        state = GetJointValuesState(joints=joints, timeout=5.0)

        userdata = _UserData()
        state.on_enter(userdata)

        # Verify on_enter initialised correctly
        assert state._joint_values == [None, None, None]

        # Build a JointState message with all three joints
        msg = JointState()
        msg.name = ['joint_a', 'joint_b', 'joint_c']
        msg.position = [1.0, 2.0, 3.0]

        # Push into the proxy buffer
        state._sub.push(state._topic, msg)

        result = state.execute(userdata)
        assert result == 'retrieved'
        assert userdata.joint_values == [1.0, 2.0, 3.0]

        state.on_exit(userdata)

    def test_incremental_partial_messages(self):
        """Joints arrive across multiple partial messages."""
        joints = ['j1', 'j2', 'j3']
        state = GetJointValuesState(joints=joints, timeout=5.0)

        userdata = _UserData()
        state.on_enter(userdata)

        # First message has only j1 and j3
        msg1 = JointState()
        msg1.name = ['j1', 'j3']
        msg1.position = [10.0, 30.0]
        state._sub.push(state._topic, msg1)

        result = state.execute(userdata)
        # Not all joints found yet
        assert result is None

        # j1 and j3 should be filled, j2 still None
        assert state._joint_values[0] == 10.0
        assert state._joint_values[1] is None
        assert state._joint_values[2] == 30.0

        # Second message has j2 (and also j1 with a DIFFERENT value – must NOT overwrite)
        msg2 = JointState()
        msg2.name = ['j1', 'j2']
        msg2.position = [99.0, 20.0]
        state._sub.push(state._topic, msg2)

        result = state.execute(userdata)
        assert result == 'retrieved'
        # j1 must still be 10.0 (first seen), not 99.0
        assert userdata.joint_values == [10.0, 20.0, 30.0]

        state.on_exit(userdata)

    def test_buffer_drain_multiple_messages_single_tick(self):
        """Multiple messages in buffer are all processed in one execute tick."""
        joints = ['a', 'b']
        state = GetJointValuesState(joints=joints, timeout=5.0)

        userdata = _UserData()
        state.on_enter(userdata)

        # Push two messages at once
        msg1 = JointState()
        msg1.name = ['a']
        msg1.position = [1.5]
        msg2 = JointState()
        msg2.name = ['b']
        msg2.position = [2.5]

        state._sub.push(state._topic, msg1)
        state._sub.push(state._topic, msg2)

        # Single execute call should drain both
        result = state.execute(userdata)
        assert result == 'retrieved'
        assert userdata.joint_values == [1.5, 2.5]

        state.on_exit(userdata)

    def test_timeout(self):
        """State returns 'timeout' when joints are not found in time."""
        joints = ['x', 'y']
        # Very short timeout
        state = GetJointValuesState(joints=joints, timeout=0.0)

        userdata = _UserData()
        state.on_enter(userdata)

        # Don't push any messages – just wait a tiny bit so clock advances
        time.sleep(0.01)

        result = state.execute(userdata)
        assert result == 'timeout'

        state.on_exit(userdata)

    def test_enable_buffer_called(self):
        """Verify enable_buffer is called during on_enter."""
        joints = ['q1']
        state = GetJointValuesState(joints=joints, timeout=5.0)

        userdata = _UserData()
        state.on_enter(userdata)

        assert state._sub._buffer_enabled.get(state._topic, False) is True

        state.on_exit(userdata)

    def test_dynamic_mapping_with_extra_joints(self):
        """Message contains extra joints not in the request – they must be ignored."""
        joints = ['target']
        state = GetJointValuesState(joints=joints, timeout=5.0)

        userdata = _UserData()
        state.on_enter(userdata)

        msg = JointState()
        msg.name = ['extra1', 'target', 'extra2']
        msg.position = [100.0, 42.0, 200.0]
        state._sub.push(state._topic, msg)

        result = state.execute(userdata)
        assert result == 'retrieved'
        assert userdata.joint_values == [42.0]

        state.on_exit(userdata)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])