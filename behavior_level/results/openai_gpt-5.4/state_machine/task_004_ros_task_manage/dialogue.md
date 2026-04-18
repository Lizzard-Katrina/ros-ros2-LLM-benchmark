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

import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from task_manager_lib.TaskClient import *
import smach
import smach_ros


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
        self.node = getattr(self.mi, 'node', None)

    def execute(self, userdata):
        self.node = getattr(self.mi, 'node', None)
        self.id = None

        try:
            if self.mi.is_shutdown():
                if self.node:
                    self.node.get_logger().info("Mission shutdown requested before starting task %s" % self.name)
                return 'TASK_INTERRUPTED'

            if self.node:
                self.node.get_logger().info("Starting task %s" % self.name)

            start_fn = None
            if hasattr(self.tc, 'startTask'):
                start_fn = self.tc.startTask
            elif hasattr(self.tc, 'addTask'):
                start_fn = self.tc.addTask
            elif hasattr(self.tc, '__call__'):
                start_fn = self.tc
            else:
                raise RuntimeError("TaskClient has no known task start method")

            result = start_fn(self.name, **self.params)

            task_id = None
            if isinstance(result, dict):
                task_id = result.get('id', None)
            elif hasattr(result, 'id'):
                task_id = result.id
            elif isinstance(result, int):
                task_id = result

            self.id = task_id

            poll_period = 0.05
            while rclpy.ok():
                if self.mi.is_shutdown() or self.preempt_requested():
                    if self.node:
                        self.node.get_logger().info("Preempting task %s" % self.name)
                    self.request_preempt()
                    self.service_preempt()
                    return 'TASK_INTERRUPTED'

                status = None
                if self.id is not None:
                    if hasattr(self.tc, 'isCompleted'):
                        try:
                            if self.tc.isCompleted(self.id):
                                status = 'TASK_COMPLETED'
                        except Exception:
                            pass

                    if status is None and hasattr(self.tc, 'getTaskStatus'):
                        try:
                            status_obj = self.tc.getTaskStatus(self.id)
                            if isinstance(status_obj, str):
                                status = status_obj
                            elif hasattr(status_obj, 'status'):
                                status = status_obj.status
                            elif hasattr(status_obj, 'state'):
                                status = status_obj.state
                        except Exception:
                            pass

                    if status is None and hasattr(self.tc, 'waitTask'):
                        try:
                            wait_result = self.tc.waitTask(self.id, poll_period)
                            if wait_result:
                                if isinstance(wait_result, str):
                                    status = wait_result
                                elif hasattr(wait_result, 'status'):
                                    status = wait_result.status
                                else:
                                    status = 'TASK_COMPLETED'
                        except TypeError:
                            pass
                        except Exception:
                            pass

                if status is not None:
                    status_str = str(status).upper()
                    if 'MISSION_COMPLETED' in status_str:
                        return 'MISSION_COMPLETED'
                    if 'TIMEOUT' in status_str:
                        return 'TASK_TIMEOUT'
                    if 'INTERRUPT' in status_str or 'CANCEL' in status_str or 'PREEMPT' in status_str:
                        return 'TASK_INTERRUPTED'
                    if 'FAIL' in status_str or 'ERROR' in status_str:
                        return 'TASK_FAILED'
                    if 'COMPLETE' in status_str or 'SUCCESS' in status_str:
                        return 'TASK_COMPLETED'

                time.sleep(poll_period)

            return 'TASK_INTERRUPTED'
        except Exception as e:
            if self.node:
                self.node.get_logger().error("Task %s failed: %s" % (self.name, str(e)))
            return 'TASK_FAILED'
        finally:
            self.id = None

    def request_preempt(self):
        if self.id:
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, tc=None, new_outcomes=[], period=0.2):
        self.shutdown_requested = False
        self.pseudo_states = {}

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('mission_state_machine')

        server_node = self.node.get_parameter_or('server', '/turtlesim_tasks').value
        default_period = self.node.get_parameter_or('period', period).value

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
        self.node.get_logger().info("Starting mission state machine")

        executor = MultiThreadedExecutor()
        executor.add_node(self.node)

        spin_stop = threading.Event()

        def spin_thread_fn():
            while rclpy.ok() and not spin_stop.is_set():
                executor.spin_once(timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin_thread_fn)
        spin_thread.daemon = True
        spin_thread.start()

        old_sigint = signal.getsignal(signal.SIGINT)
        handler = self.signal_handler(self, sm)
        signal.signal(signal.SIGINT, handler)

        sis = None
        try:
            try:
                sis = smach_ros.IntrospectionServer('mission_state_machine', sm, '/SM_ROOT')
                sis.start()
            except Exception:
                sis = None

            outcome = sm.execute()
            self.node.get_logger().info("Mission finished with outcome %s" % str(outcome))
            return outcome
        finally:
            self.shutdown_requested = True
            try:
                sm.request_preempt()
            except Exception:
                pass

            signal.signal(signal.SIGINT, old_sigint)

            if sis is not None:
                try:
                    sis.stop()
                except Exception:
                    pass

            spin_stop.set()
            try:
                spin_thread.join(timeout=1.0)
            except Exception:
                pass

            try:
                executor.remove_node(self.node)
            except Exception:
                pass

            try:
                executor.shutdown()
            except Exception:
                pass

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
                self.mi.node.get_logger().info(str(states))
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