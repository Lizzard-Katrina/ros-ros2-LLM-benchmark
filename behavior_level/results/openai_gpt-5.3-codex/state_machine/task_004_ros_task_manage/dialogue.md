# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
import roslib; roslib.load_manifest('task_manager_lib')
from task_manager_lib.TaskClient import *
import smach
import smach_ros
import signal
import sys


class TaskState(smach.State):
    def __init__(self,mi,tc,name,**params):
        smach.State.__init__(self, 
                outcomes=['TASK_COMPLETED','TASK_INTERRUPTED',
                    'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED']) # TODO: see how to add new outcomes
        self.mi = mi
        self.tc = tc
        self.name = name
        self.params = params
        self.id = None

    def execute(self, userdata):
        # TODO: Implement the task execution logic using the TaskClient (self.tc).
        # Ensure that task status (success, failure, timeout, etc.) is correctly 
        # mapped to SMACH outcomes while maintaining compatibility with the 
        # ROS 2 asynchronous execution model.
        # [STYLE]: No type hints. Use self.node for logging.
        #END OF TODO
    def request_preempt(self):
        if self.id:
            # print "Preempting task %s:%d"%(self.name,self.id)
            self.tc.stopTask(self.id)

class MissionStateMachine:
    def __init__(self,tc=None, new_outcomes=[], period = 0.2):
        self.shutdown_requested = False
        self.pseudo_states={}
        server_node = rospy.get_param("~server","/turtlesim_tasks") # FIXME: why turtlesim_tasks appear here ?
        default_period = rospy.get_param("~period", period)
        if tc:
            self.tc = tc
        else:
            self.tc = TaskClient(server_node,default_period)
        # self.tc.verbose = 2

        # Create list of default outcomes
        # ----------------------------------
        default_outcomes = ['TASK_COMPLETED','TASK_INTERRUPTED',
                    'TASK_FAILED','TASK_TIMEOUT','MISSION_COMPLETED']
        for outcome in new_outcomes:
            default_outcomes.append(outcome)
        self.default_outcomes = default_outcomes

    def is_shutdown(self):
        return self.shutdown_requested


    # Generate new name
    # --------------------
    def getLabel(self,name):
        state_name = "__"+name+"_0"
        if name in self.pseudo_states:
            state_name = "__" + name + "_" + str(self.pseudo_states[name])
        else:
            self.pseudo_states[name] = 0
        self.pseudo_states[name] += 1
        return state_name


    # Launch mission
    # -----------------
    class signal_handler:
        def __init__(self,mi,sm):
            self.mi = mi
            self.sm = sm

        def __call__(self,signal,frame):
            # print("Signal %s detected" % str(signal))
            self.mi.shutdown_requested = True
            self.sm.request_preempt()

    def run(self,sm):
        # TODO: Implement the mission execution lifecycle. This must handle 
        # concurrent execution of the State Machine and the ROS 2 Node's 
        # internal communication (callbacks/services) without deadlocking. 
        # Properly manage system signals (SIGINT) to allow for clean preemption.
        # [STYLE]: No type hints. Mandatory use of self.node handle.
        #END OF TODO

    #############################
    #####  Smach surcharge  #####
    #############################

    # State Machine
    # ----------------
    def StateMachine(self, outcomes = [], input_keys = [], output_keys = []):
        return self.StateMachineC(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys)

    class StateMachineC(smach.StateMachine):
        """ Smach StateMachine surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], input_keys = [], output_keys = []):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.StateMachine.__init__(self, outcomes = temp_outcomes, input_keys = input_keys, output_keys = output_keys)

        def add(self, label, state = None, transitions = None, remapping = None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.StateMachine.add(label, state, transitions, remapping)
            return label


    # Concurrence
    # --------------
    def Concurrence(self, outcomes = [], default_outcome = 'TASK_FAILED', input_keys = [], output_keys = [], 
                outcome_map = {}, outcome_cb = None, child_termination_cb = None, wait_for_all = True):
        return self.ConcurrenceC(self, outcomes = outcomes, default_outcome = default_outcome,
                    input_keys = input_keys, output_keys = output_keys, outcome_map = outcome_map,
                    outcome_cb = outcome_cb, child_termination_cb = child_termination_cb,
                    wait_for_all = wait_for_all)

    class ConcurrenceC(smach.Concurrence):
        """ Smach Concurrence surcharge

        Hint for performance:
        ------------------------
        You could set "foreground = True" to the longest task of the concurrence,
        it will avoid the start of Idle, which slow down the switching of task
        """
        def __init__(self, mi, outcomes = [], default_outcome = 'TASK_FAILED', input_keys = [], output_keys = [], 
                    outcome_map = {}, outcome_cb = None, child_termination_cb = None, wait_for_all = True):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            if not outcome_cb:
                outcome_cb = self.concurrent_default_outcome_cb(self.mi)
            if not child_termination_cb:
                if wait_for_all:
                    new_termination_cb = lambda x:False
                else:
                    new_termination_cb = lambda x:True
            else:
                new_termination_cb = child_termination_cb

            smach.Concurrence.__init__(self, outcomes = temp_outcomes, default_outcome = default_outcome,
                    input_keys = input_keys, output_keys = output_keys, outcome_map = outcome_map,
                    outcome_cb = outcome_cb, child_termination_cb = new_termination_cb)

        def add(self, label, state = None, remapping = None, **params):
            # if not params['foreground']:
            if not 'foreground' in params:
                params['foreground'] = False
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Concurrence.add(label, state, remapping)
            return label

        class concurrent_default_outcome_cb:
            def __init__(self,mi):
                self.mi = mi
            def __call__(self,states):
                print states
                if self.mi.is_shutdown():
                    return 'TASK_INTERRUPTED'
                num_complete = sum([1 for x in states.values() if x == 'TASK_COMPLETED'])
                if len(states) == num_complete: 
                    return 'TASK_COMPLETED'
                return 'TASK_FAILED'


    # Sequence
    # -----------
    def Sequence(self, outcomes = [], connector_outcome = 'TASK_COMPLETED', input_keys = [], output_keys = []):
        return self.SequenceC(self, outcomes = outcomes, connector_outcome = connector_outcome, 
                    input_keys = input_keys, output_keys = output_keys)

    class SequenceC(smach.Sequence):
        """ Smach Sequence surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], connector_outcome = 'TASK_COMPLETED', input_keys = [], output_keys = []):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Sequence.__init__(self, outcomes = temp_outcomes, connector_outcome = connector_outcome,
                        input_keys = input_keys, output_keys = output_keys)

        def add(self, label, state = None, transitions = None, remapping = None, **params):
            params['foreground'] = True
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Sequence.add(label, state, transitions, remapping)
            return label


    # Iterator
    # -----------
    def Iterator(self, outcomes = [], input_keys = [], output_keys = [], it = [], 
                it_label = 'it_data', exhausted_outcome = 'TASK_COMPLETED'):
        return self.IteratorC(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys,
                    it = it, it_label = it_label, exhausted_outcome = exhausted_outcome)

    class IteratorC(smach.Iterator):
        """ Smach Iterator surcharge

        bla bla
        """
        def __init__(self, mi, outcomes = [], input_keys = [], output_keys = [], it = [], 
                    it_label = 'it_data', exhausted_outcome = 'TASK_COMPLETED'):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
            for outcome in outcomes:
                temp_outcomes.append(outcome)
            smach.Iterator.__init__(self, outcomes = temp_outcomes, 
                        input_keys = input_keys, output_keys = output_keys,
                        it = it, it_label = it_label, exhausted_outcome = exhausted_outcome)


    # Epsilon task
    # -----------
    class TaskEpsilon(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['TASK_COMPLETED','TASK_FAILED'])
        def execute(self, userdata):
            return 'TASK_COMPLETED'

    # TODO: see what happen if we want to wait for all task to reach epsilon before returning completed
    def epsilon_task(self,label=None,transitions=None):
        if not label:
            label=self.getLabel("Epsilon")
        if transitions:
            smach.Sequence.add(label, self.TaskEpsilon(),transitions)
        else:
            smach.Sequence.add(label, self.TaskEpsilon())
        return label

----------------------------


# LLM Response

import threading
import signal
import sys

import rclpy
from rclpy.node import Node

from task_manager_lib.TaskClient import *
import smach
import smach_ros


class TaskState(smach.State):
    def __init__(self, mi, tc, name, **params):
        smach.State.__init__(self,
                outcomes=['TASK_COMPLETED', 'TASK_INTERRUPTED',
                    'TASK_FAILED', 'TASK_TIMEOUT', 'MISSION_COMPLETED'])  # TODO: see how to add new outcomes
        self.mi = mi
        self.tc = tc
        self.node = mi.node
        self.name = name
        self.params = params
        self.id = None

    def execute(self, userdata):
        self.id = None

        if self.mi.is_shutdown():
            self.node.get_logger().info("Mission shutdown requested before task start: %s" % self.name)
            return 'TASK_INTERRUPTED'

        try:
            self.id = self.tc.startTask(self.name, **self.params)
        except TypeError:
            try:
                self.id = self.tc.startTask(self.name, self.params)
            except Exception as e:
                self.node.get_logger().error("Failed to start task %s: %s" % (self.name, str(e)))
                return 'TASK_FAILED'
        except Exception as e:
            self.node.get_logger().error("Failed to start task %s: %s" % (self.name, str(e)))
            return 'TASK_FAILED'

        result_holder = {'result': None, 'error': None}

        def _wait_task():
            try:
                if hasattr(self.tc, 'waitTask'):
                    result_holder['result'] = self.tc.waitTask(self.id)
                else:
                    result_holder['result'] = True
            except Exception as e:
                result_holder['error'] = e

        wait_thread = threading.Thread(target=_wait_task)
        wait_thread.daemon = True
        wait_thread.start()

        while wait_thread.is_alive():
            if self.mi.is_shutdown() or self.preempt_requested():
                try:
                    self.tc.stopTask(self.id)
                except Exception:
                    pass
                self.service_preempt()
                wait_thread.join(timeout=1.0)
                return 'TASK_INTERRUPTED'
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if result_holder['error'] is not None:
            self.node.get_logger().error("Task %s execution error: %s" % (self.name, str(result_holder['error'])))
            return 'TASK_FAILED'

        result = result_holder['result']

        if isinstance(result, bool):
            return 'TASK_COMPLETED' if result else 'TASK_FAILED'

        if isinstance(result, tuple) and len(result) > 1:
            result = result[1]

        if isinstance(result, dict) and 'status' in result:
            result = result['status']

        if result is None:
            return 'TASK_FAILED'

        if isinstance(result, str):
            status_str = result.upper()
        elif hasattr(result, 'status'):
            status_str = str(result.status).upper()
        else:
            status_str = str(result).upper()

        if 'MISSION_COMPLETED' in status_str:
            return 'MISSION_COMPLETED'
        if 'INTERRUPT' in status_str or 'CANCEL' in status_str or 'PREEMPT' in status_str:
            return 'TASK_INTERRUPTED'
        if 'TIMEOUT' in status_str:
            return 'TASK_TIMEOUT'
        if 'COMPLETE' in status_str or 'SUCCESS' in status_str or status_str == '2':
            return 'TASK_COMPLETED'
        if 'FAIL' in status_str or 'ERROR' in status_str or status_str == '3':
            return 'TASK_FAILED'
        if status_str == '4':
            return 'TASK_TIMEOUT'
        if status_str == '5':
            return 'TASK_INTERRUPTED'

        return 'TASK_FAILED'

    def request_preempt(self):
        if self.id:
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, tc=None, new_outcomes=[], period=0.2):
        self.shutdown_requested = False
        self.pseudo_states = {}

        if not rclpy.ok():
            rclpy.init(args=None)

        self._owns_node = False
        if tc and hasattr(tc, 'node') and tc.node is not None:
            self.node = tc.node
        else:
            self.node = rclpy.create_node('mission_state_machine')
            self._owns_node = True

        self.node.declare_parameter('server', '/turtlesim_tasks')  # FIXME: why turtlesim_tasks appear here ?
        self.node.declare_parameter('period', period)

        server_node = self.node.get_parameter('server').get_parameter_value().string_value
        default_period = self.node.get_parameter('period').get_parameter_value().double_value

        if tc:
            self.tc = tc
        else:
            try:
                self.tc = TaskClient(self.node, server_node, default_period)
            except TypeError:
                try:
                    self.tc = TaskClient(server_node, default_period, node=self.node)
                except TypeError:
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
        previous_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self.signal_handler(self, sm))

        outcome_holder = {'outcome': None, 'error': None}

        def _execute_sm():
            try:
                outcome_holder['outcome'] = sm.execute()
            except Exception as e:
                outcome_holder['error'] = e

        sm_thread = threading.Thread(target=_execute_sm)
        sm_thread.daemon = True
        sm_thread.start()

        try:
            while sm_thread.is_alive():
                if self.shutdown_requested:
                    sm.request_preempt()
                rclpy.spin_once(self.node, timeout_sec=0.1)
            sm_thread.join()
        finally:
            signal.signal(signal.SIGINT, previous_handler)

        if outcome_holder['error'] is not None:
            self.node.get_logger().error("State machine execution failed: %s" % str(outcome_holder['error']))
            return 'TASK_FAILED'

        return outcome_holder['outcome']

    #############################
    #####  Smach surcharge  #####
    #############################

    def StateMachine(self, outcomes=[], input_keys=[], output_keys=[]):
        return self.StateMachineC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    class StateMachineC(smach.StateMachine):
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
            smach.StateMachine.add(label, state, transitions, remapping)
            return label

    def Concurrence(self, outcomes=[], default_outcome='TASK_FAILED', input_keys=[], output_keys=[],
                outcome_map={}, outcome_cb=None, child_termination_cb=None, wait_for_all=True):
        return self.ConcurrenceC(self, outcomes=outcomes, default_outcome=default_outcome,
                    input_keys=input_keys, output_keys=output_keys, outcome_map=outcome_map,
                    outcome_cb=outcome_cb, child_termination_cb=child_termination_cb,
                    wait_for_all=wait_for_all)

    class ConcurrenceC(smach.Concurrence):
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
            if not 'foreground' in params:
                params['foreground'] = False
            if not state:
                state = TaskState(self.mi, self.mi.tc, label, **params)
                label = self.mi.getLabel(label)
            smach.Concurrence.add(label, state, remapping)
            return label

        class concurrent_default_outcome_cb:
            def __init__(self, mi):
                self.mi = mi

            def __call__(self, states):
                self.mi.node.get_logger().debug(str(states))
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
            smach.Sequence.add(label, state, transitions, remapping)
            return label

    def Iterator(self, outcomes=[], input_keys=[], output_keys=[], it=[],
                it_label='it_data', exhausted_outcome='TASK_COMPLETED'):
        return self.IteratorC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys,
                    it=it, it_label=it_label, exhausted_outcome=exhausted_outcome)

    class IteratorC(smach.Iterator):
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