#!/usr/bin/env python
# ORIGINAL: behavior_engine__ORIGINAL.py
# Trimmed, but functionally similar to original FlexBE behavior_engine implementation.

import rospy
import threading
import time

class BehaviorEngine(object):
    def __init__(self, behavior):
        self._behavior = behavior
        self._active_states = []
        self._running = False
        self._lock = threading.RLock()
        # simple logger
        self._log = rospy.loginfo

    def start(self):
        self._running = True
        self._log("BehaviorEngine: starting")
        self._execute_thread = threading.Thread(target=self._execute)
        self._execute_thread.start()

    def stop(self):
        self._running = False
        self._log("BehaviorEngine: stopping")
        if self._execute_thread.is_alive():
            self._execute_thread.join()

    def _execute(self):
        """
        Original state execution loop:
        - iterate active states
        - call state's execute method
        - monitor for completion or timeout
        - update active state list and behavior userdata accordingly
        """
        while self._running:
            with self._lock:
                current_states = list(self._active_states)
            for state in current_states:
                try:
                    # call state execute
                    outcome = state.execute(state.userdata)
                    # handle the outcome: update behavior, plan next states etc.
                    # simplified original logic:
                    if outcome in state.outcomes:
                        self._log("State %s returned: %s" % (state.name, outcome))
                        # remove state from active list if terminal
                        if outcome == 'done' or outcome == 'success' or outcome == 'fail':
                            with self._lock:
                                if state in self._active_states:
                                    self._active_states.remove(state)
                            # there would be further wiring to push transitions to the behavior
                    else:
                        self._log("State %s returned unknown outcome: %s" % (state.name, outcome))
                except Exception as e:
                    self._log("Exception in state execute: %s" % str(e))
            time.sleep(0.05)
