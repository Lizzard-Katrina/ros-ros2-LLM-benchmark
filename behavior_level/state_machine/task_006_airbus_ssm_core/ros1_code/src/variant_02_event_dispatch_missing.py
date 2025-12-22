#!/usr/bin/env python
# VARIANT v2: event dispatch missing
# Purpose: event queue consumption and matching to transitions removed.
# Original runtime popped events and matched transitions from current node.

import rospy
import threading
import time
from ssm_core.scxml_loader import SCXMLLoader  # in real placement adjust import path

class SSMRuntimeVariantV2(object):
    def __init__(self, scxml_path):
        self.loader = SCXMLLoader(scxml_path)
        self.initial, self.nodes = self.loader.load()
        self.active_state = self.initial
        self.event_queue = []
        self._lock = threading.RLock()
        self._running = False
        self._log = rospy.loginfo

    def start(self):
        self._running = True
        self._log("SSM runtime starting with initial state: %s" % str(self.initial))
        self._exec_thread = threading.Thread(target=self._loop)
        self._exec_thread.start()
        self._event_thread = threading.Thread(target=self._process_events_loop)
        self._event_thread.start()

    def stop(self):
        self._running = False

    def post_event(self, event):
        with self._lock:
            self.event_queue.append(event)

    def _loop(self):
        while self._running:
            node = self.nodes.get(self.active_state)
            if node is None:
                self._log("Unknown active state: %s" % str(self.active_state))
                break
            # (entry logic / state actions would be here)
            time.sleep(0.1)

    def _process_events_loop(self):
        while self._running:
            ev = None
            with self._lock:
                if self.event_queue:
                    ev = self.event_queue.pop(0)

            if ev:
                # TODO
                # Implement:
                #   1) thread-safe reading of current node's transitions
                #   2) matching event by ev.name
                #   3) condition evaluation using eval() with ev.payload as namespace (use getattr safely)
                #   4) apply first matched transition: set active_state to target and log
                # END
            time.sleep(0.05)
