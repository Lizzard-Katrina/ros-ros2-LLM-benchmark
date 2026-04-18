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

```python
import rclpy
from rclpy.node import Node
import signal
import sys
import threading
import time

# Assuming task_manager_lib has been ported to ROS2
from task_manager_lib.TaskClient import *
import smach
import smach_ros2


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
        try:
            self.id = self.tc.startTask(self.name, **self.params)
            
            while not self.mi.is_shutdown():
                status = self.tc.getTaskStatus(self.id)
                
                if status == TaskStatus.SUCCEEDED:
                    return 'TASK_COMPLETED'
                elif status == TaskStatus.FAILED:
                    return 'TASK_FAILED'
                elif status == TaskStatus.TIMEOUT:
                    return 'TASK_TIMEOUT'
                elif status == TaskStatus.PREEMPTED:
                    return 'TASK_INTERRUPTED'
                elif status == TaskStatus.ABORTED:
                    return 'TASK_INTERRUPTED'
                
                time.sleep(0.01)
                
            if self.mi.is_shutdown():
                self.tc.stopTask(self.id)
                return 'TASK_INTERRUPTED'
                
        except Exception as e:
            self.node.get_logger().error(f"Task {self.name} failed: {str(e)}")
            return 'TASK_FAILED'

    def request_preempt(self):
        if self.id:
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, node, tc=None, new_outcomes=[], period=0.2):
        self.node = node
        self.shutdown_requested = False
        self.pseudo_states = {}
        
        server_node = self.node.get_parameter('server').value if self.node.has_parameter('server') else "/turtlesim_tasks"
        default_period = self.node.get_parameter('period').value if self.node.has_parameter('period') else period
        
        if tc:
            self.tc = tc
        else:
            self.tc = TaskClient(node, server_node, default_period)

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
        handler = self.signal_handler(self, sm)
        signal.signal(signal.SIGINT, handler)
        
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        
        sm_thread = threading.Thread(target=sm.execute)
        sm_thread.start()
        
        try:
            while rclpy.ok() and not self.is_shutdown() and sm_thread.is_alive():
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            self.shutdown_requested = True
            sm.request_preempt()
        
        sm_thread.join(timeout=1.0)
        executor.shutdown()

    def StateMachine(self, outcomes=[], input_keys=[], output_keys=[]):
        return self.StateMachineC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    class StateMachineC(smach.StateMachine):
        def __init__(self, mi, outcomes=[], input_keys=[], output_keys=[]):
            self.mi = mi
            temp_outcomes = self.mi.default_outcomes
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
            temp_outcomes = self.mi.default_outcomes
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
            temp_outcomes = self.mi.default_outcomes
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
            temp_outcomes = self.mi.default_outcomes
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
```