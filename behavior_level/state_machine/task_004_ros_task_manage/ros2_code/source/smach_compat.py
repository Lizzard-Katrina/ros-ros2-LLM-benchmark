"""
Minimal smach-compatible state machine classes for ROS2.
Replaces the smach / smach_ros packages which are not available on Humble.
"""
import threading
import traceback


class State:
    """Minimal smach.State replacement."""
    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        self._outcomes = list(outcomes or [])
        self._input_keys = list(input_keys or [])
        self._output_keys = list(output_keys or [])
        self._preempt_requested = False

    def execute(self, userdata):
        raise NotImplementedError

    def request_preempt(self):
        self._preempt_requested = True

    def preempt_requested(self):
        return self._preempt_requested

    def service_preempt(self):
        self._preempt_requested = False


class UserData(dict):
    """Simple userdata container."""
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        self[name] = value


class StateMachine(State):
    """Minimal smach.StateMachine replacement."""
    _currently_opened = None

    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self._states = {}
        self._transitions = {}
        self._remappings = {}
        self._state_order = []
        self._initial_state = None
        self._running = False
        self._preempt_requested = False

    def __enter__(self):
        StateMachine._currently_opened = self
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        StateMachine._currently_opened = None
        return False

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        sm = StateMachine._currently_opened
        if sm is None:
            raise RuntimeError("StateMachine.add called outside of 'with' block")
        sm._states[label] = state
        sm._transitions[label] = transitions or {}
        sm._remappings[label] = remapping or {}
        sm._state_order.append(label)
        if sm._initial_state is None:
            sm._initial_state = label

    def execute(self, userdata=None):
        if userdata is None:
            userdata = UserData()
        self._running = True
        current = self._initial_state
        try:
            while current and not self._preempt_requested:
                state = self._states[current]
                outcome = state.execute(userdata)
                if outcome in self._outcomes:
                    self._running = False
                    return outcome
                trans = self._transitions.get(current, {})
                if outcome in trans:
                    current = trans[outcome]
                else:
                    self._running = False
                    return outcome
        except Exception:
            traceback.print_exc()
            self._running = False
            return 'TASK_FAILED'
        self._running = False
        if self._preempt_requested:
            return 'TASK_INTERRUPTED'
        return 'TASK_FAILED'

    def is_running(self):
        return self._running

    def request_preempt(self):
        self._preempt_requested = True
        for state in self._states.values():
            if hasattr(state, 'request_preempt'):
                state.request_preempt()


class Sequence(StateMachine):
    """Minimal smach.Sequence replacement."""
    def __init__(self, outcomes=None, connector_outcome='TASK_COMPLETED',
                 input_keys=None, output_keys=None):
        StateMachine.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self._connector_outcome = connector_outcome

    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        sm = StateMachine._currently_opened
        if sm is None:
            raise RuntimeError("Sequence.add called outside of 'with' block")
        sm._states[label] = state
        sm._transitions[label] = transitions or {}
        sm._remappings[label] = remapping or {}
        sm._state_order.append(label)
        if sm._initial_state is None:
            sm._initial_state = label

    def execute(self, userdata=None):
        if userdata is None:
            userdata = UserData()
        self._running = True
        try:
            for label in self._state_order:
                if self._preempt_requested:
                    self._running = False
                    return 'TASK_INTERRUPTED'
                state = self._states[label]
                outcome = state.execute(userdata)
                if outcome in self._outcomes:
                    self._running = False
                    return outcome
                trans = self._transitions.get(label, {})
                if outcome in trans:
                    next_label = trans[outcome]
                    if next_label in self._outcomes:
                        self._running = False
                        return next_label
                elif outcome != self._connector_outcome:
                    self._running = False
                    return outcome
        except Exception:
            traceback.print_exc()
            self._running = False
            return 'TASK_FAILED'
        self._running = False
        return self._connector_outcome if self._connector_outcome in self._outcomes else 'TASK_COMPLETED'


class Concurrence(StateMachine):
    """Minimal smach.Concurrence replacement."""
    def __init__(self, outcomes=None, default_outcome='TASK_FAILED',
                 input_keys=None, output_keys=None, outcome_map=None,
                 outcome_cb=None, child_termination_cb=None):
        StateMachine.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self._default_outcome = default_outcome
        self._outcome_map = outcome_map or {}
        self._outcome_cb = outcome_cb
        self._child_termination_cb = child_termination_cb

    def execute(self, userdata=None):
        if userdata is None:
            userdata = UserData()
        self._running = True
        results = {}
        threads = []

        def run_state(label, state):
            try:
                results[label] = state.execute(userdata)
            except Exception:
                results[label] = 'TASK_FAILED'

        for label in self._state_order:
            t = threading.Thread(target=run_state, args=(label, self._states[label]))
            threads.append(t)
            t.start()

        for t in threads:
            t.join(timeout=30)

        self._running = False

        if self._outcome_cb:
            return self._outcome_cb(results)
        return self._default_outcome


class Iterator(StateMachine):
    """Minimal smach.Iterator replacement."""
    def __init__(self, outcomes=None, input_keys=None, output_keys=None,
                 it=None, it_label='it_data', exhausted_outcome='TASK_COMPLETED'):
        StateMachine.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self._it = it or []
        self._it_label = it_label
        self._exhausted_outcome = exhausted_outcome


class IntrospectionServer:
    """Minimal smach_ros.IntrospectionServer replacement (no-op for ROS2)."""
    def __init__(self, name, node, sm, path):
        self.name = name
        self.node = node
        self.sm = sm
        self.path = path

    def start(self):
        pass

    def stop(self):
        pass