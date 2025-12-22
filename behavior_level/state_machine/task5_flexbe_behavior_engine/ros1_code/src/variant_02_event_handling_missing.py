import rospy
import threading

class BehaviorEngineVariantV2(object):
    def __init__(self, behavior):
        self._behavior = behavior
        self._event_queue = []
        self._lock = threading.RLock()
        self._log = rospy.loginfo

    def post_event(self, event):
        with self._lock:
            self._event_queue.append(event)

    def _handle_event(self, event):
        # TODO
        # The original engine processed events by:
        #   - inspecting event.type and event.payload
        #   - mapping the event to one or more state transitions in the behavior
        #   - updating userdata or pushing the next state into _active_states
        # The LLM must implement a handler that:
        #   1) safely consumes the event
        #   2) modifies self._behavior or self._active_states accordingly
        #   3) uses locking to be thread-safe
        # END

    def _process_events_loop(self):
        while True:
            event = None
            with self._lock:
                if self._event_queue:
                    event = self._event_queue.pop(0)
            if event is None:
                # no event to process: sleep briefly to avoid busy-waiting
                rospy.sleep(0.05)
                continue
            # process event outside lock to avoid long blocking
            try:
                self._handle_event(event)
            except Exception as e:
                self._log("Exception while processing event: %s" % str(e))
