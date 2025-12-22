#!/usr/bin/env python
# path: task2_mas_execution_manager/ros1_code/nodes/execution_manager.py

import rospy
import smach
import smach_ros
import actionlib

from mas_exec_manager.execution_manager_core import LoadTask, ExecuteTask, CheckResult, Recovery, Finished
from mas_exec_manager.action_client_wrapper import ActionExecutor
from mas_exec_manager.msg import TaskRequestAction  # assumed action type


def main():
    rospy.init_node("execution_manager_node")

    # Create action client wrapper
    rospy.loginfo("Execution Manager: creating action client")
    client = ActionExecutor()

    rospy.loginfo("Execution Manager: building state machine")
    sm = smach.StateMachine(outcomes=['DONE'])
    sm.userdata.task_id = None
    sm.userdata.retry_count = 0

    with sm:
        # Add states (original, full version)
        smach.StateMachine.add('LOAD_TASK', LoadTask(),
                               transitions={'loaded': 'EXECUTE_TASK',
                                            'no_task': 'DONE'},
                               remapping={'task_id': 'task_id'})

        smach.StateMachine.add('EXECUTE_TASK', ExecuteTask(client),
                               transitions={'succ': 'CHECK_RESULT',
                                            'fail': 'RECOVERY'},
                               remapping={'task_id': 'task_id'})

        smach.StateMachine.add('CHECK_RESULT', CheckResult(),
                               transitions={'ok': 'LOAD_TASK',
                                            'done': 'FINISHED',
                                            'retry': 'EXECUTE_TASK'},
                               remapping={'task_id': 'task_id', 'retry_count': 'retry_count'})

        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'recovered': 'LOAD_TASK',
                                            'abort': 'DONE'},
                               remapping={'task_id': 'task_id'})

        smach.StateMachine.add('FINISHED', Finished(),
                               transitions={'done': 'DONE'})

    rospy.loginfo("Execution Manager: starting state machine")
    outcome = sm.execute()
    rospy.loginfo("Execution Manager finished with outcome: %s", outcome)


if __name__ == '__main__':
    main()
