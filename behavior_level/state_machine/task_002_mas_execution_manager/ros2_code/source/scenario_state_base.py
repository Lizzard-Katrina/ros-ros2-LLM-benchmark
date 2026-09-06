import smach
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

try:
    import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
except ImportError:
    plan_dispatch_msgs = None

try:
    from mdr_monitoring_msgs.msg import ExecutionState
except ImportError:
    ExecutionState = None

try:
    from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
except ImportError:
    DomesticOntologyInterface = None

try:
    from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface
except ImportError:
    DomesticKBInterface = None


class ScenarioStateBase(smach.State):
    def __init__(self, node, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.node = node
        self.sm_id = ''
        self.state_name = ''
        self.action_name = action_name
        self.save_sm_state = save_sm_state
        self.retry_count = 0
        self.executing = False
        self.succeeded = False

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.say_pub = self.node.create_publisher(String, '/say', qos_profile=latching_qos)

        self.node.declare_parameter('ontology_url', '')
        self.node.declare_parameter('ontology_class_prefix', '')

        self.ontology_url = self.node.get_parameter('ontology_url').value
        self.ontology_class_prefix = self.node.get_parameter('ontology_class_prefix').value

        if plan_dispatch_msgs is not None:
            self.action_dispatch_pub = self.node.create_publisher(
                plan_dispatch_msgs.ActionDispatch,
                '/kcl_rosplan/action_dispatch',
                10
            )

            self.node.create_subscription(
                plan_dispatch_msgs.ActionFeedback,
                '/kcl_rosplan/action_feedback',
                self.get_action_feedback,
                10
            )

        if DomesticKBInterface is not None:
            self.kb_interface = DomesticKBInterface()
        else:
            self.kb_interface = None

        if DomesticOntologyInterface is not None:
            self.ontology_interface = DomesticOntologyInterface(self.ontology_url,
                                                                self.ontology_class_prefix)
        else:
            self.ontology_interface = None

        if self.kb_interface is not None:
            self.robot_name = self.kb_interface.robot_name
        else:
            self.robot_name = ''

    def execute(self, userdata):
        pass

    def save_current_state(self):
        if ExecutionState is None:
            return
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