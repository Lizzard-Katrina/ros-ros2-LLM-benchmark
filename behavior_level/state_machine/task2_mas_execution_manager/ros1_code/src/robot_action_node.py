import rospy
import actionlib

from mas_exec_manager.msg import TaskRequestAction, TaskRequestResult

class TaskServer(object):
    def __init__(self):
        # Initialize the action server
        self.server = actionlib.SimpleActionServer(
            "/task_request",
            TaskRequestAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[TaskServer] Action server '/task_request' started.")

    def execute(self, goal):
        """
        Execute callback for the action server.
        Expects goal.task_id (int) and simulates different behaviors per task_id.
        Handles preemption and exceptions, and reports success/failure accordingly.
        """
        rospy.loginfo("[TaskServer] Received goal: task_id=%s" % str(getattr(goal, 'task_id', 'unknown')))

        result = TaskRequestResult()
        try:
            # Pre-check for preemption before starting
            if self.server.is_preempt_requested():
                rospy.loginfo("[TaskServer] Preempt requested before start.")
                result.success = False
                self.server.set_preempted(result, "Preempted before start")
                return

            task_id = int(getattr(goal, 'task_id', 0))

            # Simulated task execution for pick (1) and place (2)
            if task_id == 1:
                rospy.loginfo("[TaskServer] Starting PICK task simulation.")
                # simulate several steps; check preemption during the steps
                for step in range(3):
                    if self.server.is_preempt_requested():
                        rospy.loginfo("[TaskServer] Preempt requested during PICK.")
                        result.success = False
                        self.server.set_preempted(result, "Preempted during PICK")
                        return
                    rospy.loginfo("[TaskServer] PICK step %d/3" % (step+1))
                    rospy.sleep(0.6)  # simulate processing

                # simulate success condition
                result.success = True
                rospy.loginfo("[TaskServer] PICK completed successfully.")
                self.server.set_succeeded(result)
                return

            elif task_id == 2:
                rospy.loginfo("[TaskServer] Starting PLACE task simulation.")
                # simulate navigation + place sequence
                for step in range(4):
                    if self.server.is_preempt_requested():
                        rospy.loginfo("[TaskServer] Preempt requested during PLACE.")
                        result.success = False
                        self.server.set_preempted(result, "Preempted during PLACE")
                        return
                    rospy.loginfo("[TaskServer] PLACE step %d/4" % (step+1))
                    rospy.sleep(0.5)

                # simulate possible intermittent failure (for realism)
                # here we'll make it succeed; in tests you can modify to fail
                result.success = True
                rospy.loginfo("[TaskServer] PLACE completed successfully.")
                self.server.set_succeeded(result)
                return

            else:
                rospy.logwarn("[TaskServer] Unknown task_id=%s. Aborting." % str(task_id))
                result.success = False
                self.server.set_aborted(result, "Unknown task_id")
                return

        except Exception as e:
            rospy.logerr("[TaskServer] Exception during execution: %s" % str(e))
            result.success = False
            try:
                self.server.set_aborted(result, "Exception: %s" % str(e))
            except Exception:
                # If set_aborted itself fails, just log and return
                rospy.logerr("[TaskServer] Failed to set_aborted due to exception.")
            return


if __name__ == "__main__":
    rospy.init_node("task_server")
    server = TaskServer()
    rospy.spin()
