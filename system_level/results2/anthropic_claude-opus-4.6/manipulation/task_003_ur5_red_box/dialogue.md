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

FILE_PATH: set_joint_states_client.py
----------------------------
#!/usr/bin/env python

import sys
import rospy
from robotic_arm_algorithms.srv import *

def set_joint_states(joint_states):
# TODO: Implement a robust ROS 2 service client. 
# The client must handle the service call asynchronously to prevent 
# blocking the main executor, and it should gracefully wait for 
# the response before exiting.
# END OF TODO

if __name__ == "__main__":
    if len(sys.argv) == 5:
        joint_states = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
        set_joint_states(joint_states)
    else:
        print("not enaugh argument. Four arguments required: forearm 0, forearm 1, arm 0, arm 1")
        sys.exit(1)
        

----------------------------

FILE_PATH: set_joint_states_service.py
----------------------------
#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32
from moveit_commander.conversions import pose_to_list
from robotic_arm_algorithms.srv import *


class MoveItContext(object):
    """
    MoveIt! context object contains all information about the robot and planning.
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instance of a `RobotCommander`_ object provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantce of a `PlanningSceneInterface`_ object provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantce of a `MoveGroupCommander`_ object is an interface
        ## to a planning group (group of joints).  
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names


    def go_to_joint_state(self, req):
        print("Starting planning to go to joint state.\n")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = req.forearm_0.data   # forearm 0
        joint_goal[1] = req.forearm_1.data   # forearm 1
        joint_goal[2] = req.arm_0.data   # arm 0
        joint_goal[3] = req.arm_1.data   # arm 1

        print("Setting joint goal:")
        print(joint_goal)
        print("")

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

# TODO: Refactor this ROS 1 service node into a ROS 2 Node. 
# The service callback must adhere to the ROS 2 standard signature 
# to process the target joint states and confirm execution success. 
# Ensure the node remains responsive while waiting for the motion to complete.
def set_joint_states(req):

def set_joint_states_server():
# END OF TODO

if __name__ == '__main__':
    try:
        set_joint_states_server()
    except rospy.ROSInterruptException:
        pass

----------------------------


# LLM Response

[FILENAME: set_joint_states_client.py]
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from robotic_arm_algorithms.srv import SetJointStates


def set_joint_states(joint_states):
    rclpy.init()
    node = Node('set_joint_states_client')
    client = node.create_client(SetJointStates, 'set_joint_states')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    request = SetJointStates.Request()
    from std_msgs.msg import Float32
    request.forearm_0 = Float32(data=joint_states[0])
    request.forearm_1 = Float32(data=joint_states[1])
    request.arm_0 = Float32(data=joint_states[2])
    request.arm_1 = Float32(data=joint_states[3])

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Service call succeeded: %s' % str(future.result().success))
    else:
        node.get_logger().error('Service call failed: %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 5:
        joint_states = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
        set_joint_states(joint_states)
    else:
        print("not enaugh argument. Four arguments required: forearm 0, forearm 1, arm 0, arm 1")
        sys.exit(1)

[FILENAME: set_joint_states_service.py]
#!/usr/bin/env python3

import sys
import copy
import rclpy
from rclpy.node import Node
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32
from moveit_commander.conversions import pose_to_list
from robotic_arm_algorithms.srv import SetJointStates


class MoveItContext(object):
    """
    MoveIt! context object contains all information about the robot and planning.
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instance of a `RobotCommander`_ object provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantce of a `PlanningSceneInterface`_ object provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantce of a `MoveGroupCommander`_ object is an interface
        ## to a planning group (group of joints).  
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names


    def go_to_joint_state(self, req):
        print("Starting planning to go to joint state.\n")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = req.forearm_0.data   # forearm 0
        joint_goal[1] = req.forearm_1.data   # forearm 1
        joint_goal[2] = req.arm_0.data   # arm 0
        joint_goal[3] = req.arm_1.data   # arm 1

        print("Setting joint goal:")
        print(joint_goal)
        print("")

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()


class SetJointStatesServiceNode(Node):
    def __init__(self):
        super().__init__('set_joint_states_server')
        self.moveit_context = MoveItContext()
        self.srv = self.create_service(SetJointStates, 'set_joint_states', self.set_joint_states_callback)
        self.get_logger().info('Set joint states service is ready.')

    def set_joint_states_callback(self, request, response):
        self.get_logger().info('Received request to set joint states.')
        try:
            self.moveit_context.go_to_joint_state(request)
            response.success = True
        except Exception as e:
            self.get_logger().error('Failed to set joint states: %s' % str(e))
            response.success = False
        return response


def set_joint_states(req):
    pass


def set_joint_states_server():
    rclpy.init(args=sys.argv)
    node = SetJointStatesServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        set_joint_states_server()
    except KeyboardInterrupt:
        pass