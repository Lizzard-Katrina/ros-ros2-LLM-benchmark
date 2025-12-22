#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
from mas_execution_manager.msg import ExecuteAction, ExecuteGoal

class ExecState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['task_id'],
            output_keys=['result_msg']
        )
        self.client = actionlib.SimpleActionClient(
            '/mas_execution_manager/execute',
            ExecuteAction
        )
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo("ExecState: Executing task {}".format(userdata.task_id))

        goal = ExecuteGoal()
        goal.task_id = userdata.task_id

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        userdata.result_msg = result.result_message

        # TODO: Implement failure recovery logic.
        # Missing logic should:
        #   - Inspect result.success
        #   - If failure, attempt recovery (e.g., retry, fallback, message check)
        #   - Return proper transition based on recovery outcome
        #
        # END

class TaskManagerStateMachine(object):
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['DONE'])

        with self.sm:
            smach.StateMachine.add(
                'EXECUTE_TASK',
                ExecState(),
                transitions={
                    'success': 'DONE',
                    'failure': 'DONE'
                }
            )

    def run(self):
        outcome = self.sm.execute()
        rospy.loginfo("State machine finished with outcome: {}".format(outcome))

if __name__ == '__main__':
    rospy.init_node('task_manager_node')
    manager = TaskManagerStateMachine()
    manager.run()
