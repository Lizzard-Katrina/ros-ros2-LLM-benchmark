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
        # TODO: Determine next state based on result status
        # The missing logic should choose which outcome the state returns.
        # LLM should decide the correct transition outcome given:
        #   - result.success (bool)
        #   - result.result_message (string)
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
