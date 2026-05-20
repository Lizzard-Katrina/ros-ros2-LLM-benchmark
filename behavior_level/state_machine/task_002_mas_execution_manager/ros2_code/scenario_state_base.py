import rclpy
import smach
from std_msgs.msg import String
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs

from mdr_monitoring_msgs.msg import ExecutionState
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface
from rclpy.qos import QoSProfile, DurabilityPolicy

class ScenarioStateBase(smach.State):
    def __init__(self, node, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.node = node
        self.action_name = action_name
        self.save_sm_state = save_sm_state
        self.state_name = self.__class__.__name__
        self.sm_id = self.node.get_name()
        
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.say_pub = self.node.create_publisher(String, '/say', latching_qos)
        
        self.ontology_interface = DomesticOntologyInterface(self.node)
        self.kb_interface = DomesticKBInterface(self.node)
        
        self.executing = False
        self.succeeded = False

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