# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
import rospy
import smach
from std_msgs.msg import String
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs

from mdr_monitoring_msgs.msg import ExecutionState
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface

class ScenarioStateBase(smach.State):
    def __init__(self, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        # TODO: [ROS 2 MIGRATION - ARCHITECTURAL TASK]
        # 1. Update the state to be ROS 2 compatible by accepting a node handle.
        # 2. Re-establish all communication interfaces (Publishers, Subscriptions, and Parameters) 
        #    using the ROS 2 rclpy API while strictly preserving the original ROS 1 topic 
        #    names and business logic.
        # 3. Ensure external interfaces (KB/Ontology) are initialized with the same 
        #    parameter values as the original source.
        #
        # [CONSTRAINTS & STYLE]:
        # - The constructor must accept 'node' as the first argument after 'self'.
        # - DO NOT use Python type hints (e.g., use 'node', NOT 'node: Node').
        # - Store the node handle as 'self.node'.
        # - Replicate the 'latching' behavior for the '/say' topic using ROS 2 QoS.
        #END OF TODO
    def execute(self, userdata):
        pass

    def save_current_state(self):
        execution_state_msg = ExecutionState()
        execution_state_msg.stamp = rospy.Time.now()
        execution_state_msg.state_machine = self.sm_id
        execution_state_msg.state = self.state_name
        self.kb_interface.insert_obj_instance('current_state', execution_state_msg)

    def get_dispatch_msg(self):
        pass

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = msg.status == 'action achieved'

    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)

----------------------------


# LLM Response

 ```python
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
import smach
from std_msgs.msg import String
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs

from mdr_monitoring_msgs.msg import ExecutionState
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface

class ScenarioStateBase(smach.State):
    def __init__(self, node, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        self.node = node
        self.action_name = action_name
        self.sm_id = ""
        self.state_name = ""
        self.executing = False
        self.succeeded = False
        
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=input_keys,
                            output_keys=output_keys)
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.say_pub = self.node.create_publisher(String, '/say', qos_profile)
        
        self.kb_interface = DomesticKBInterface()
        self.ontology_interface = DomesticOntologyInterface()
        
        self.sub = self.node.create_subscription(
            plan_dispatch_msgs.ActionFeedback,
            '/rosplan_plan_dispatcher/action_feedback',
            self.get_action_feedback,
            10)
        
        self.save_sm_state = save_sm_state

    def execute(self, userdata):
        pass

    def save_current_state(self):
        execution_state_msg = ExecutionState()
        execution_state_msg.stamp = self.node.get_clock().now().to_msg()
        execution_state_msg.state_machine = self.sm_id
        execution_state_msg.state = self.state_name
        self.kb_interface.insert_obj_instance('current_state', execution_state_msg)

    def get_dispatch_msg(self):
        pass

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = msg.status == 'action achieved'

    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)
```