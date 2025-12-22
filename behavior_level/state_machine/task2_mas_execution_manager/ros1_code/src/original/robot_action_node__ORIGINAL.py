#!/usr/bin/env python
# ORIGINAL: robot_action_node.py

import rospy
import actionlib
from mas_exec_manager.msg import TaskRequestAction, TaskRequestResult

class TaskServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/task_request",
            TaskRequestAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("TaskServer: ready.")

    def execute(self, goal):
        rospy.loginfo("TaskServer: received goal: %s" % str(goal.task_id))

        # simple simulated execution
        result = TaskRequestResult()
        try:
            if goal.task_id == 1:
                rospy.loginfo("Simulating PICK task")
                rospy.sleep(1.0)  # simulate action
                result.success = True
                self.server.set_succeeded(result)
            elif goal.task_id == 2:
                rospy.loginfo("Simulating PLACE task")
                rospy.sleep(1.0)
                result.success = True
                self.server.set_succeeded(result)
            else:
                rospy.logwarn("Unknown task_id, aborting")
                result.success = False
                self.server.set_aborted(result)
        except Exception as e:
            rospy.logerr("Exception in TaskServer: %s" % str(e))
            result.success = False
            self.server.set_aborted(result)


if __name__ == "__main__":
    rospy.init_node("task_server")
    TaskServer()
    rospy.spin()
