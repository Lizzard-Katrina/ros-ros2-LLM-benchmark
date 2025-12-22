#!/usr/bin/env python
# path: task2_mas_execution_manager/ros1_code/src/mas_exec_manager/execution_manager_core.py

import rospy
import smach
import random


class LoadTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loaded', 'no_task'],
                             output_keys=['task_id'])
        # In a real system this might subscribe to a task queue or param server

    def execute(self, userdata):
        rospy.loginfo("[LoadTask] Checking for next task")
        # Simulate reading a list of tasks from param server
        tasks = rospy.get_param("/mas_exec_manager/tasks", ["pick", "place"])
        if not tasks:
            rospy.loginfo("[LoadTask] No tasks found")
            return 'no_task'
        # pick first task for simplicity
        task = tasks.pop(0)
        rospy.set_param("/mas_exec_manager/tasks", tasks)
        userdata.task_id = 1 if task == "pick" else 2
        rospy.loginfo("[LoadTask] Loaded task: %s (id=%d)" % (task, userdata.task_id))
        return 'loaded'


class ExecuteTask(smach.State):
    def __init__(self, action_client):
        smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['task_id'])
        self.client = action_client

    def execute(self, userdata):
        rospy.loginfo("[ExecuteTask] Executing task id=%s" % str(userdata.task_id))
        # send goal using action client wrapper
        try:
            ok = self.client.run(userdata.task_id)
            rospy.loginfo("[ExecuteTask] Action execution returned: %s" % str(ok))
            return 'succ' if ok else 'fail'
        except Exception as e:
            rospy.logerr("[ExecuteTask] Exception during action execution: %s" % str(e))
            return 'fail'


class CheckResult(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok', 'done', 'retry'],
                             input_keys=['task_id', 'retry_count'],
                             output_keys=['retry_count'])

    def execute(self, userdata):
        rospy.loginfo("[CheckResult] Checking results for task id=%s" % str(userdata.task_id))
        # Placeholder decision logic: randomly decide to retry or mark done for demonstration
        r = random.random()
        if r < 0.6:
            rospy.loginfo("[CheckResult] Task ok, continue with next")
            return 'ok'
        elif r < 0.9:
            # request retry
            userdata.retry_count = userdata.retry_count + 1 if userdata.retry_count else 1
            rospy.loginfo("[CheckResult] Request retry, count=%d" % userdata.retry_count)
            return 'retry'
        else:
            rospy.loginfo("[CheckResult] Everything done")
            return 'done'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recovered', 'abort'],
                             input_keys=['task_id'])

    def execute(self, userdata):
        rospy.loginfo("[Recovery] Attempting recovery for task id=%s" % str(userdata.task_id))
        # simple retry logic or escalate
        retry_attempts = rospy.get_param("/mas_exec_manager/retry_attempts", 2)
        current = rospy.get_param("/mas_exec_manager/current_retry", 0)
        rospy.set_param("/mas_exec_manager/current_retry", current + 1)
        if current < retry_attempts:
            rospy.loginfo("[Recovery] Recovery succeeded, will retry")
            return 'recovered'
        else:
            rospy.logwarn("[Recovery] Recovery failed, aborting")
            return 'abort'


class Finished(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[Finished] All tasks complete")
        return 'done'
