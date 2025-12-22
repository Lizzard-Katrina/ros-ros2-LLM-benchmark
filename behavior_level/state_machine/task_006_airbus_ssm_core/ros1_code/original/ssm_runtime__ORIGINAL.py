#!/usr/bin/env python
# ORIGINAL ssm_runtime: simplified runtime that uses SCXML loader, maintains active state, event queue, transition eval
import rospy
import threading
import time
from scxml_loader import SCXMLLoader

class SSMRuntime(object):
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
        # main loop: check current active_state, run onentry logs, and wait for events to trigger transitions
        while self._running:
            node = self.nodes.get(self.active_state)
            if node is None:
                self._log("Unknown active state: %s" % str(self.active_state))
                break
            # entry action (log)
            # waiting for events - transitions evaluated by event handler
            time.sleep(0.1)

    def _process_events_loop(self):
        while self._running:
            ev = None
            with self._lock:
                if self.event_queue:
                    ev = self.event_queue.pop(0)
            if ev:
                # find transitions from current state that match the event
                node = self.nodes.get(self.active_state, {})
                trans = node.get('transitions', [])
                matched = []
                for t in trans:
                    if t['event'] == getattr(ev, 'name', None):
                        # condition evaluation (if cond present)
                        cond = t.get('cond')
                        ok = True
                        if cond:
                            # basic eval - here original may use more complex engine
                            ok = eval(cond, {}, getattr(ev, 'payload', {}))
                        if ok:
                            matched.append(t)
                if matched:
                    # apply first matched transition
                    tgt = matched[0]['target']
                    self._log("Transitioning from %s to %s on event %s" % (self.active_state, tgt, ev.name))
                    self.active_state = tgt
            time.sleep(0.05)
