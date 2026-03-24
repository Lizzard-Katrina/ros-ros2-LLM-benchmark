import rclpy
from rclpy.node import Node
from task_manager_lib.TaskClient import TaskClient
import smach
import smach_ros
import signal
import sys
import threading


class TaskState(smach.State):
    def __init__(self, mi, tc, name, **params):
        smach.State.__init__(self,
                             outcomes=['TASK_COMPLETED', 'TASK_INTERRUPTED',
                                       'TASK_FAILED', 'TASK_TIMEOUT', 'MISSION_COMPLETED'])
        self.mi = mi
        self.tc = tc
        self.name = name
        self.params = params
        self.id = None

    def execute(self, userdata):
        self.mi.node.get_logger().info(f"Starting task {self.name}")
        # Start the task using TaskClient
        self.id = self.tc.startTask(self.name, **self.params)

        # Wait for task completion or interruption
        while rclpy.ok():
            status = self.tc.getTaskStatus(self.id)
            if status == TaskClient.STATUS_SUCCEEDED:
                self.mi.node.get_logger().info(f"Task {self.name} completed successfully")
                return 'TASK_COMPLETED'
            elif status == TaskClient.STATUS_PREEMPTED:
                self.mi.node.get_logger().info(f"Task {self.name} was interrupted")
                return 'TASK_INTERRUPTED'
            elif status == TaskClient.STATUS_ABORTED:
                self.mi.node.get_logger().info(f"Task {self.name} failed")
                return 'TASK_FAILED'
            elif status == TaskClient.STATUS_TIMEOUT:
                self.mi.node.get_logger().info(f"Task {self.name} timed out")
                return 'TASK_TIMEOUT'
            elif status == TaskClient.STATUS_MISSION_COMPLETED:
                self.mi.node.get_logger().info(f"Mission completed during task {self.name}")
                return 'MISSION_COMPLETED'
            # Sleep a bit to avoid busy waiting
            rclpy.sleep(0.1)

        # If ROS is shutdown externally
        self.mi.node.get_logger().info(f"Task {self.name} interrupted by shutdown")
        return 'TASK_INTERRUPTED'

    def request_preempt(self):
        if self.id:
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, tc=None, new_outcomes=[], period=0.2):
        self.shutdown_requested = False
        self.pseudo_states = {}

        rclpy.init(args=None)
        self.node = Node('mission_state_machine')

        server_node = self.node.get_parameter_or('server', '/turtlesim_tasks')
        default_period = self.node.get_parameter_or('period', period)

        if tc:
            self.tc = tc
        else:
            self.tc = TaskClient(server_node, default_period)

        default_outcomes = ['TASK_COMPLETED', 'TASK_INTERRUPTED',
                            'TASK_FAILED', 'TASK_TIMEOUT', 'MISSION_COMPLETED']
        for outcome in new_outcomes:
            default_outcomes.append(outcome)
        self.default_outcomes = default_outcomes

    def is_shutdown(self):
        return self.shutdown_requested

    def getLabel(self, name):
        if name in self.pseudo_states:
            state_name = "__" + name + "_" + str(self.pseudo_states[name])
        else:
            self.pseudo_states[name] = 0
            state_name = "__" + name + "_0"
        self.pseudo_states[name] += 1
        return state_name

    class signal_handler:
        def __init__(self, mi, sm):
            self.mi = mi
            self.sm = sm

        def __call__(self, signum, frame):
            self.mi.shutdown_requested = True
            self.sm.request_preempt()

    def run(self, sm):
        # Create a separate thread to spin the ROS2 node
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)

        def spin_thread():
            while rclpy.ok() and not self.shutdown_requested:
                executor.spin_once(timeout_sec=0.1)

        thread = threading.Thread(target=spin_thread, daemon=True)
        thread.start()

        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler(self, sm))

        # Execute the state machine in the main thread
        outcome = None
        try:
            outcome = sm.execute()
        except KeyboardInterrupt:
            self.node.get_logger().info("KeyboardInterrupt received, shutting down")
            self.shutdown_requested = True
            sm.request_preempt()
            outcome = 'TASK_INTERRUPTED'

        # Shutdown ROS2 node and executor
        self.shutdown_requested = True
        thread.join()
        executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

        return outcome

    #############################
    #####  Smach surcharge  #####
    #############################

    def StateMachine(self, outcomes=[], input_keys=[], output_keys=[]):
        return self.StateMachineC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    class StateMachineC(smach.StateMachine):
        """ Smach StateMachine surcharge

        bla bla
        """
        def __init__(self, mi, outcomes=[], input_keys=[], output_keys=[]):
            self.mi = mi
            temp_outcomes = list(self.mi.default_outcomes)
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.StateMachine.__init__(self, outcomes=temp_outcomes, input_keys=input_keys, output_keys=output_keys)

        def add(self, label, state=None, transitions=None, remapping=None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.StateMachine.add(self, label, state, transitions, remapping)
            return label

    def Concurrence(self, outcomes=[], default_outcome='TASK_FAILED', input_keys=[], output_keys=[],
                    outcome_map={}, outcome_cb=None, child_termination_cb=None, wait_for_all=True):
        return self.ConcurrenceC(self, outcomes=outcomes, default_outcome=default_outcome,
                                 input_keys=input_keys, output_keys=output_keys, outcome_map=outcome_map,
                                 outcome_cb=outcome_cb, child_termination_cb=child_termination_cb,
                                 wait_for_all=wait_for_all)

    class ConcurrenceC(smach.Concurrence):
        """ Smach Concurrence surcharge

        Hint for performance:
        ------------------------
        You could set "foreground = True" to the longest task of the concurrence,
        it will avoid the start of Idle, which slow down the switching of task
        """
        def __init__(self, mi, outcomes=[], default_outcome='TASK_FAILED', input_keys=[], output_keys=[],
                     outcome_map={}, outcome_cb=None, child_termination_cb=None, wait_for_all=True):
            self.mi = mi
            temp_outcomes = list(self.mi.default_outcomes)
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            if not outcome_cb:
                outcome_cb = self.concurrent_default_outcome_cb(self.mi)
            if not child_termination_cb:
                if wait_for_all:
                    new_termination_cb = lambda x: False
                else:
                    new_termination_cb = lambda x: True
            else:
                new_termination_cb = child_termination_cb

            smach.Concurrence.__init__(self, outcomes=temp_outcomes, default_outcome=default_outcome,
                                      input_keys=input_keys, output_keys=output_keys, outcome_map=outcome_map,
                                      outcome_cb=outcome_cb, child_termination_cb=new_termination_cb)

        def add(self, label, state=None, remapping=None, **params):
            if 'foreground' not in params:
                params['foreground'] = False
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Concurrence.add(self, label, state, remapping)
            return label

        class concurrent_default_outcome_cb:
            def __init__(self, mi):
                self.mi = mi

            def __call__(self, states):
                print(states)
                if self.mi.is_shutdown():
                    return 'TASK_INTERRUPTED'
                num_complete = sum([1 for x in states.values() if x == 'TASK_COMPLETED'])
                if len(states) == num_complete:
                    return 'TASK_COMPLETED'
                return 'TASK_FAILED'

    def Sequence(self, outcomes=[], connector_outcome='TASK_COMPLETED', input_keys=[], output_keys=[]):
        return self.SequenceC(self, outcomes=outcomes, connector_outcome=connector_outcome,
                              input_keys=input_keys, output_keys=output_keys)

    class SequenceC(smach.Sequence):
        """ Smach Sequence surcharge

        bla bla
        """
        def __init__(self, mi, outcomes=[], connector_outcome='TASK_COMPLETED', input_keys=[], output_keys=[]):
            self.mi = mi
            temp_outcomes = list(self.mi.default_outcomes)
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Sequence.__init__(self, outcomes=temp_outcomes, connector_outcome=connector_outcome,
                                    input_keys=input_keys, output_keys=output_keys)

        def add(self, label, state=None, transitions=None, remapping=None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Sequence.add(self, label, state, transitions, remapping)
            return label

    def Iterator(self, outcomes=[], input_keys=[], output_keys=[], it=[],
                 it_label='it_data', exhausted_outcome='TASK_COMPLETED'):
        return self.IteratorC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys,
                              it=it, it_label=it_label, exhausted_outcome=exhausted_outcome)

    class IteratorC(smach.Iterator):
        """ Smach Iterator surcharge

        bla bla
        """
        def __init__(self, mi, outcomes=[], input_keys=[], output_keys=[], it=[],
                     it_label='it_data', exhausted_outcome='TASK_COMPLETED'):
            self.mi = mi
            temp_outcomes = list(self.mi.default_outcomes)
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Iterator.__init__(self, outcomes=temp_outcomes,
                                    input_keys=input_keys, output_keys=output_keys,
                                    it=it, it_label=it_label, exhausted_outcome=exhausted_outcome)

    class TaskEpsilon(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

        def execute(self, userdata):
            return 'TASK_COMPLETED'

    def epsilon_task(self, label=None, transitions=None):
        if not label:
            label = self.getLabel("Epsilon")
        if transitions:
            smach.Sequence.add(self, label, self.TaskEpsilon(), transitions)
        else:
            smach.Sequence.add(self, label, self.TaskEpsilon())
        return label