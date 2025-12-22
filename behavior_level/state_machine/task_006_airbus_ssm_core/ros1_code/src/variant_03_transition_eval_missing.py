import rospy
import threading
import time
from ssm_core.scxml_loader import SCXMLLoader

class SSMRuntimeVariantV3(object):
    def __init__(self, scxml_path):
        self.loader = SCXMLLoader(scxml_path)
        self.initial, self.nodes = self.loader.load()
        self.active_state = self.initial
        self._lock = threading.RLock()
        self._running = False
        self._log = rospy.loginfo

    def start(self):
        self._running = True
        self._log("SSM runtime starting with initial state: %s" % str(self.initial))
        self._event_thread = threading.Thread(target=self._process_events_loop)
        self._event_thread.start()

    def stop(self):
        self._running = False

    def post_event(self, event):
        # immediate processing in this variant for simplicity
        with self._lock:
            node = self.nodes.get(self.active_state, {})
            trans = node.get('transitions', [])
            for t in trans:
                if t['event'] == getattr(event, 'name', None):
                    # TODO
                    # Implement condition evaluation (use safe getattr on payload), log and apply transition
                    # END
