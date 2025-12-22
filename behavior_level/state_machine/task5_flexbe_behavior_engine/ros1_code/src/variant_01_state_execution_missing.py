import rospy
import threading
import time

class BehaviorEngineVariant(object):
    def __init__(self, behavior):
        self._behavior = behavior
        self._active_states = []
        self._running = False
        self._lock = threading.RLock()
        self._log = rospy.loginfo

    def start(self):
        self._running = True
        self._log("BehaviorEngineVariant: starting")
        self._execute_thread = threading.Thread(target=self._execute)
        self._execute_thread.start()

    def stop(self):
        self._running = False
        self._log("BehaviorEngineVariant: stopping")
        if self._execute_thread.is_alive():
            self._execute_thread.join()

    def _execute(self):
        # TODO
        # The original code iterated active states and executed them:
        # Restore: call state.execute, handle successes/failures, and remove terminal states.
        # END
