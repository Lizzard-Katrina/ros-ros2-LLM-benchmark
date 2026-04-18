import rclpy
import smach
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs

from mdr_monitoring_msgs.msg import ExecutionState
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface


class ScenarioStateBase(smach.State):
    def __init__(self, node, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

        self.node = node
        self.action_name = action_name
        self.save_sm_state = save_sm_state
        self.executing = False
        self.succeeded = False

        self.node.declare_parameter('sm_id', '')
        self.node.declare_parameter('state_name', self.__class__.__name__)
        self.sm_id = self.node.get_parameter('sm_id').value
        self.state_name = self.node.get_parameter('state_name').value

        self.node.declare_parameter('action_feedback_topic', '/kcl_rosplan/action_feedback')
        action_feedback_topic = self.node.get_parameter('action_feedback_topic').value

        say_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.say_pub = self.node.create_publisher(String, '/say', say_qos)

        self.action_feedback_sub = self.node.create_subscription(
            plan_dispatch_msgs.ActionFeedback,
            action_feedback_topic,
            self.get_action_feedback,
            10
        )

        self.node.declare_parameter('ontology_query_service_name', '/domestic_ontology/query')
        self.node.declare_parameter('ontology_update_service_name', '/domestic_ontology/update')
        self.node.declare_parameter('kb_query_service_name', '/domestic_kb/query')
        self.node.declare_parameter('kb_update_service_name', '/domestic_kb/update')
        self.node.declare_parameter('kb_insert_service_name', '/domestic_kb/insert')

        ontology_query_service_name = self.node.get_parameter('ontology_query_service_name').value
        ontology_update_service_name = self.node.get_parameter('ontology_update_service_name').value
        kb_query_service_name = self.node.get_parameter('kb_query_service_name').value
        kb_update_service_name = self.node.get_parameter('kb_update_service_name').value
        kb_insert_service_name = self.node.get_parameter('kb_insert_service_name').value

        try:
            self.ontology_interface = DomesticOntologyInterface(
                ontology_query_service_name,
                ontology_update_service_name
            )
        except TypeError:
            try:
                self.ontology_interface = DomesticOntologyInterface(
                    query_service_name=ontology_query_service_name,
                    update_service_name=ontology_update_service_name
                )
            except TypeError:
                self.ontology_interface = DomesticOntologyInterface()

        try:
            self.kb_interface = DomesticKBInterface(
                kb_query_service_name,
                kb_update_service_name,
                kb_insert_service_name
            )
        except TypeError:
            try:
                self.kb_interface = DomesticKBInterface(
                    query_service_name=kb_query_service_name,
                    update_service_name=kb_update_service_name,
                    insert_service_name=kb_insert_service_name
                )
            except TypeError:
                self.kb_interface = DomesticKBInterface()

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