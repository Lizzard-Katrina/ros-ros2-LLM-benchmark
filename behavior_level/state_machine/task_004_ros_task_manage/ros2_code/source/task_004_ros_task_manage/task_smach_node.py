import rclpy
import sys
import os

# Add parent directory so TaskSmach.py can be found
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from TaskSmach import MissionStateMachine, TaskState, TaskStatus, TaskException, TaskConditionException
import smach


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    node = rclpy.create_node('task_smach_test_node')
    node.get_logger().info('TaskSmach node started')

    # Create a simple mock TaskClient for standalone testing
    class MockTaskDef:
        def __init__(self, name):
            self.name = name
        def start(self, **params):
            return 1

    class MockTaskClient:
        def __init__(self):
            self.tasklist = {'MockTask': MockTaskDef('MockTask')}
        def waitTask(self, task_id):
            pass
        def stopTask(self, task_id=None):
            pass

    mock_tc = MockTaskClient()

    mi = MissionStateMachine(tc=mock_tc, node=node)

    # Create a simple state machine
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_INTERRUPTED',
                                      'TASK_FAILED', 'TASK_TIMEOUT', 'MISSION_COMPLETED'])

    with sm:
        task_state = TaskState(mi, mock_tc, 'MockTask', foreground=True)
        smach.StateMachine.add('MOCK_TASK', task_state,
                               transitions={'TASK_COMPLETED': 'MISSION_COMPLETED',
                                            'TASK_INTERRUPTED': 'MISSION_COMPLETED',
                                            'TASK_FAILED': 'MISSION_COMPLETED',
                                            'TASK_TIMEOUT': 'MISSION_COMPLETED',
                                            'MISSION_COMPLETED': 'MISSION_COMPLETED'})

    node.get_logger().info('State machine created, executing...')
    outcome = sm.execute()
    node.get_logger().info('State machine outcome: ' + str(outcome))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()