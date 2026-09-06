import rclpy
from rclpy.executors import MultiThreadedExecutor
import smach_compat as smach
from smach_compat import IntrospectionServer
import signal
import threading
import time
import sys


class TaskException(Exception):
    def __init__(self, value, id=None, status=None, statusString=""):
        self.id = id
        self.value = value
        self.status = status
        self.statusString = statusString
    def __str__(self):
        return repr(self.value)


class TaskConditionException(Exception):
    def __init__(self, value, conds=None):
        self.value = value
        self.conditions = conds
    def __str__(self):
        return repr(self.value)


class TaskStatus:
    TASK_COMPLETED = 0
    TASK_RUNNING = 1
    TASK_FAILED = 2
    TASK_TIMEOUT = 3
    TASK_INTERRUPTED = 4


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
        self.node = mi.node

    def execute(self, userdata):
        if self.mi.is_shutdown():
            return 'MISSION_COMPLETED'
        try:
            self.node.get_logger().info('Executing state ' + self.name)
            self.node.get_logger().info('Params: ' + str(self.params))
            self.id = self.tc.tasklist[self.name].start(**self.params)
            self.tc.waitTask(self.id)
            return 'TASK_COMPLETED'
        except TaskConditionException:
            return 'TASK_INTERRUPTED'
        except TaskException as e:
            if e.status == TaskStatus.TASK_TIMEOUT:
                return 'TASK_TIMEOUT'
            elif e.status == TaskStatus.TASK_INTERRUPTED:
                return 'TASK_INTERRUPTED'
            return 'TASK_FAILED'

    def request_preempt(self):
        if self.id:
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, tc=None, new_outcomes=[], period=0.2, node=None):
        if not rclpy.ok():
            rclpy.init()
        self.shutdown_requested = False
        self.pseudo_states = {}

        if node is not None:
            self.node = node
        else:
            self.node = rclpy.create_node('mission_state_machine_node')

        server_node = self.node.declare_parameter('server', '/turtlesim_tasks').get_parameter_value().string_value
        default_period = self.node.declare_parameter('period', period).get_parameter_value().double_value

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
        state_name = "__" + name + "_0"
        if name in self.pseudo_states:
            state_name = "__" + name + "_" + str(self.pseudo_states[name])
        else:
            self.pseudo_states[name] = 0
        self.pseudo_states[name] += 1
        return state_name

    class signal_handler:
        def __init__(self, mi, sm):
            self.mi = mi
            self.sm = sm

        def __call__(self, signal, frame):
            self.mi.shutdown_requested = True
            self.sm.request_preempt()

    def run(self, sm):
        self.shutdown_requested = False
        self.node.get_logger().info('Starting mission state machine')

        sis = IntrospectionServer('mission_state_machine', self.node, sm, '/SM')
        sis.start()

        signal.signal(signal.SIGINT, self.signal_handler(self, sm))

        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        smach_thread = threading.Thread(target=sm.execute)
        smach_thread.start()

        while sm.is_running() and rclpy.ok():
            time.sleep(0.5)

        smach_thread.join(timeout=5.0)

        sis.stop()
        executor.shutdown()
        self.node.get_logger().info('Mission state machine finished')

    #############################
    #####  Smach surcharge  #####
    #############################

    def StateMachine(self, outcomes=[], input_keys=[], output_keys=[]):
        return self.StateMachineC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    class StateMachineC(smach.StateMachine):
        """ Smach StateMachine surcharge """
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
            with self:
                smach.StateMachine.add(label, state, transitions, remapping)
            return label

    def Concurrence(self, outcomes=[], default_outcome='TASK_FAILED', input_keys=[], output_keys=[],
                outcome_map={}, outcome_cb=None, child_termination_cb=None, wait_for_all=True):
        return self.ConcurrenceC(self, outcomes=outcomes, default_outcome=default_outcome,
                    input_keys=input_keys, output_keys=output_keys, outcome_map=outcome_map,
                    outcome_cb=outcome_cb, child_termination_cb=child_termination_cb,
                    wait_for_all=wait_for_all)

    class ConcurrenceC(smach.Concurrence):
        """ Smach Concurrence surcharge """
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
            with self:
                smach.Concurrence.add(label, state, remapping)
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
        """ Smach Sequence surcharge """
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
            with self:
                smach.Sequence.add(label, state, transitions, remapping)
            return label

    def Iterator(self, outcomes=[], input_keys=[], output_keys=[], it=[],
                it_label='it_data', exhausted_outcome='TASK_COMPLETED'):
        return self.IteratorC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys,
                    it=it, it_label=it_label, exhausted_outcome=exhausted_outcome)

    class IteratorC(smach.Iterator):
        """ Smach Iterator surcharge """
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
            smach.Sequence.add(label, self.TaskEpsilon(), transitions)
        else:
            smach.Sequence.add(label, self.TaskEpsilon())
        return label