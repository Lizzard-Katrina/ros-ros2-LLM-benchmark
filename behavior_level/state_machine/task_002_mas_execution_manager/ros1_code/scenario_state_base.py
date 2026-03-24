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
