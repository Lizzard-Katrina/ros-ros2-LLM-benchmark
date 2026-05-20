# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: scenario_state_base.py
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

[FILENAME: scenario_state_base.py]
import rclpy
import smach
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs

from mdr_monitoring_msgs.msg import ExecutionState
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface

class ScenarioStateBase(smach.State):
    def __init__(self, node, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)

        self.node = node
        self.action_name = action_name
        self.save_sm_state = save_sm_state

        self.executing = False
        self.succeeded = False

        self.state_name = self.__class__.__name__

        self.node.declare_parameter('sm_id', 'default')
        self.node.declare_parameter('ontology_url', '')
        self.node.declare_parameter('ontology_topic', '/ontology')
        self.node.declare_parameter('kb_server_name', 'knowledge_base')

        self.sm_id = self.node.get_parameter('sm_id').value
        ontology_url = self.node.get_parameter('ontology_url').value
        ontology_topic = self.node.get_parameter('ontology_topic').value
        kb_server_name = self.node.get_parameter('kb_server_name').value

        self.ontology_interface = DomesticOntologyInterface(ontology_url, ontology_topic)
        self.kb_interface = DomesticKBInterface(kb_server_name)

        say_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.say_pub = self.node.create_publisher(String, '/say', say_qos)
        self.dispatch_pub = self.node.create_publisher(
            plan_dispatch_msgs.ActionDispatch,
            '/kcl_rosplan/action_dispatch',
            10
        )
        self.feedback_sub = self.node.create_subscription(
            plan_dispatch_msgs.ActionFeedback,
            '/kcl_rosplan/action_feedback',
            self.get_action_feedback,
            10
        )

    def execute(self, userdata):
        pass

    def save_current_state(self):
        execution_state_msg = ExecutionState()
        execution_state_msg.stamp = self.node.get_clock().now().to_msg()
        execution_state_msg.state_machine = self.sm_id
        execution_state_msg.state = self.state_name
        self.kb_interface.insert_obj_instance('current_state', execution_state_msg)

    def get_dispatch_msg(self):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name
        return dispatch_msg

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = msg.status == 'action achieved'

    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)