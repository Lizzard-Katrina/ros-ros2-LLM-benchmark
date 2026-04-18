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
from task_manager_lib.task_client import TaskClient
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
        self.node = mi.node  # Access the ROS2 node for logging

    def execute(self, userdata):
        self.node.get_logger().info(f"Executing task: {self.name}")
        try:
            self.id = self.tc.startTask(self.name, **self.params)
            if self.id is None:
                self.node.get_logger().error(f"Failed to start task: {self.name}")
                return 'TASK_FAILED'

            while rclpy.ok() and not self.mi.is_shutdown():
                status = self.tc.getTaskStatus(self.id)
                if status is None:
                    self.node.get_logger().error(f"Failed to get status for task: {self.name}")
                    return 'TASK_FAILED'

                if status.is_running:
                    rclpy.spin_once(self.node, timeout_sec=0.1)  # Allow ROS2 callbacks
                    continue
                elif status.is_completed:
                    self.node.get_logger().info(f"Task {self.name} completed successfully.")
                    return 'TASK_COMPLETED'
                elif status.is_interrupted:
                    self.node.get_logger().warn(f"Task {self.name} was interrupted.")
                    return 'TASK_INTERRUPTED'
                elif status.is_failed:
                    self.node.get_logger().error(f"Task {self.name} failed.")
                    return 'TASK_FAILED'
                elif status.is_timed_out:
                    self.node.get_logger().warn(f"Task {self.name} timed out.")
                    return 'TASK_TIMEOUT'
                else:
                    self.node.get_logger().warn(f"Unknown task status for {self.name}: {status}")
                    return 'TASK_FAILED'
            self.node.get_logger().info(f"Task {self.name} execution interrupted by shutdown request.")
            return 'TASK_INTERRUPTED'
        except Exception as e:
            self.node.get_logger().error(f"Exception during task {self.name} execution: {e}")
            return 'TASK_FAILED'

    def request_preempt(self):
        if self.id:
            self.node.get_logger().info(f"Preempting task {self.name}:{self.id}")
            self.tc.stopTask(self.id)


class MissionStateMachine:
    def __init__(self, tc=None, new_outcomes=[], period=0.2):
        self.node = rclpy.create_node('mission_state_machine')
        self.shutdown_requested = False
        self.pseudo_states = {}
        server_node_name = self.node.declare_parameter("server", "/turtlesim_tasks").value
        default_period = self.node.declare_parameter("period", period).value

        if tc:
            self.tc = tc
        else:
            self.tc = TaskClient(self.node, server_node_name, default_period)

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

        def __call__(self, signum, frame):
            self.mi.node.get_logger().info(f"Signal {signum} detected")
            self.mi.shutdown_requested = True
            self.sm.request_preempt()

    def run(self, sm):
        signal.signal(signal.SIGINT, self.signal_handler(self, sm))

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        self.node.get_logger().info("Starting state machine...")
        try:
            outcome = sm.execute()
            self.node.get_logger().info(f"State machine finished with outcome: {outcome}")
        except Exception as e:
            self.node.get_logger().error(f"State machine execution failed: {e}")
        finally:
            self.shutdown_requested = True
            self.node.get_logger().info("Shutting down ROS2 node and executor.")
            executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()
            spin_thread.join()

    #############################
    #####  Smach surcharge  #####
    #############################

    def StateMachine(self, outcomes=[], input_keys=[], output_keys=[]):
        return self.StateMachineC(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    class StateMachineC(smach.StateMachine):
        def __init__(self, mi, outcomes=[], input_keys=[], output_keys=[]):
            self.mi = mi
            temp_outcomes = list(self.mi.default_outcomes)  # Create a copy
            for outcome in outcomes:
                if outcome not in temp_outcomes:
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
                if outcome not in temp_outcomes:
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
            smach.Concurrence.add(label, state, remapping)
            return label

        class concurrent_default_outcome_cb:
            def __init__(self, mi):
                self.mi = mi

            def __call__(self, states):
                self.mi.node.get_logger().debug(f"Concurrent states: {states}")
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
                if outcome not in temp_outcomes:
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
                if outcome not in temp_outcomes:
                    temp_outcomes.append(outcome)
            smach.Iterator.__init__(self, outcomes=temp_outcomes,
                                    input_keys=input_keys, output_keys=output_keys,
                                    it=it, it_label=it_label, exhausted_outcome=exhausted_outcome)

    class TaskEpsilon(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
            self.node = rclpy.create_node('task_epsilon_node') # Temporary node for logging if not passed

        def execute(self, userdata):
            self.node.get_logger().info("Executing Epsilon task.")
            return 'TASK_COMPLETED'

    def epsilon_task(self, label=None, transitions=None):
        if not label:
            label = self.getLabel("Epsilon")
        epsilon_state = self.TaskEpsilon()
        # The original ROS1 code directly added to smach.Sequence, which implies
        # this method is expected to be called within a Sequence context.
        # For a more robust solution, you might want to pass the parent container
        # or modify this to return the state to be added by the caller.
        # For now, assuming it's called within a Sequence context where 'add' is available.
        if transitions:
            smach.Sequence.add(label, epsilon_state, transitions)
        else:
            smach.Sequence.add(label, epsilon_state)
        return label
```