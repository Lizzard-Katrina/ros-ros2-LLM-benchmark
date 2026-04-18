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

FILE_PATH: controller.py
----------------------------
import math
import copy
import rospy
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
from pyquaternion import Quaternion


def get_controller_state(controller_topic, timeout=None):
    return rospy.wait_for_message(
        f"{controller_topic}/state",
        control_msgs.msg.JointTrajectoryControllerState,
        timeout=timeout)


class ArmController:
    def __init__(self, gripper_state=0, controller_topic="/trajectory_controller"):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.gripper_state = gripper_state

        self.controller_topic = controller_topic
        self.default_joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        self.default_joint_trajectory.joint_names = self.joint_names

        joint_states = get_controller_state(controller_topic).actual.positions
        x, y, z, rot = kinematics.get_pose(joint_states)
        self.gripper_pose = (x, y, z), Quaternion(matrix=rot)

        # Create an action client for the joint trajectory
        self.joints_pub = rospy.Publisher(
            f"{self.controller_topic}/command",
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        """
    	TODO:
    	Execute an end-effector motion to the target pose by generating
    	and publishing joint trajectories, and update the internal arm state
    	based on execution feedback.
    	END OF TODO
        """


    def send_joints(self, x, y, z, quat, duration=1.0):  # x,y,z and orientation of lego block
        # Solve for the joint angles, select the 5th solution
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states
            pts.velocities = [0, 0, 0, 0, 0, 0]
            pts.time_from_start = rospy.Time(duration)
            # Set the points to the trajectory
            traj.points = [pts]
            # Publish the message
            self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            msg = get_controller_state(self.controller_topic, timeout=10)
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                    return
        rospy.logwarn("Timeout waiting for position")

----------------------------

FILE_PATH: motion_planning.py
----------------------------
#!/usr/bin/python3

import os
import math
import copy
import json
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "X1-Y2-Z1": {
        "home": [0.264589, -0.293903, 0.777] 
    },
    "X2-Y2-Z2": {
        "home": [0.277866, -0.724482, 0.777] 
    },
    "X1-Y3-Z2": {
        "home": [0.268053, -0.513924, 0.777]  
    },
    "X1-Y2-Z2": {
        "home": [0.429198, -0.293903, 0.777] 
    },
    "X1-Y2-Z2-CHAMFER": {
        "home": [0.592619, -0.293903, 0.777]  
    },
    "X1-Y4-Z2": {
        "home": [0.108812, -0.716057, 0.777] 
    },
    "X1-Y1-Z2": {
        "home": [0.088808, -0.295820, 0.777] 
    },
    "X1-Y2-Z2-TWINFILLET": {
        "home": [0.103547, -0.501132, 0.777] 
    },
    "X1-Y3-Z2-FILLET": {
        "home": [0.433739, -0.507130, 0.777]  
    },
    "X1-Y4-Z1": {
        "home": [0.589908, -0.501033, 0.777]  
    },
    "X2-Y2-Z2-FILLET": {
        "home": [0.442505, -0.727271, 0.777] 
    }
}

for model, model_info in MODELS_INFO.items():
    pass
    #MODELS_INFO[model]["home"] = model_info["home"] + np.array([0.0, 0.10, 0.0])

for model, info in MODELS_INFO.items():
    model_json_path = os.path.join(PKG_PATH, "..", "models", f"lego_{model}", "model.json")
    # make path absolute
    model_json_path = os.path.abspath(model_json_path)
    # check path exists
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = (np.max(corners[:, 0]) - np.min(corners[:, 0]))
    size_y = (np.max(corners[:, 1]) - np.min(corners[:, 1]))
    size_z = (np.max(corners[:, 2]) - np.min(corners[:, 2]))

    #print(f"{model}: {size_x:.3f} x {size_y:.3f} x {size_z:.3f}")

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]


def get_legos_pos(vision=False):
    #get legos position reading vision topic
    if vision:
        legos = rospy.wait_for_message("/lego_detections", ModelStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        legos = ModelStates()

        for name, pose in zip(models.name, models.pose):
            if "X" not in name:
                continue
            name = get_model_name(name)

            legos.name.append(name)
            legos.pose.append(pose)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]


def straighten(model_pose, gazebo_model_name):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)

    model_size = MODELS_INFO[get_model_name(gazebo_model_name)]["size"]

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = get_axis_facing_camera(model_quat)
    approach_angle = get_approach_angle(model_quat, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")

    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)

    # Get above the object
    controller.move_to(x, y, target_quat=approach_quat)

    # Calculate target quat
    regrip_quat = DEFAULT_QUAT
    if facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):  # Side
        target_quat = DEFAULT_QUAT
        pitch_angle = -math.pi/2 + 0.2

        if abs(approach_angle) < math.pi/2:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        else:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
        target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat

        if facing_direction == (0, 1, 0):
            regrip_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) * regrip_quat

    elif facing_direction == (0, 0, -1):
        """
            Pre-positioning
        """
        controller.move_to(z=z, target_quat=approach_quat)
        close_gripper(gazebo_model_name, model_size[0])

        tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2*math.pi/6) * DEFAULT_QUAT
        controller.move_to(SAFE_X, SAFE_Y, z+0.05, target_quat=tmp_quat, z_raise=0.1)  # Move to safe position
        controller.move_to(z=z)
        open_gripper(gazebo_model_name)

        approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)

        target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)  # Add a yaw rotation of 180 deg

        regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

    """
        Grip the model
    """
    if facing_direction == (0, 0, 1) or facing_direction == (0, 0, -1):
        closure = model_size[0]
        z = SURFACE_Z + model_size[2] / 2
    elif facing_direction == (1, 0, 0):
        closure = model_size[1]
        z = SURFACE_Z + model_size[0] / 2
    elif facing_direction == (0, 1, 0):
        closure = model_size[0]
        z = SURFACE_Z + model_size[1] / 2
    controller.move_to(z=z, target_quat=approach_quat)
    close_gripper(gazebo_model_name, closure)

    """
        Straighten model if needed
    """
    if facing_direction != (0, 0, 1):
        z = SURFACE_Z + model_size[2]/2

        controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
        controller.move(dz=-0.05)
        open_gripper(gazebo_model_name)

        # Re grip the model
        controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
        close_gripper(gazebo_model_name, model_size[0])


def close_gripper(gazebo_model_name, closure=0):
    set_gripper(0.81-closure*10)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)


def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi/2:
            yaw_angle = math.pi/2
        else:
            yaw_angle = -math.pi/2
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat


def get_axis_facing_camera(quat):
    axis_x = np.array([1, 0, 0])
    axis_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])
    new_axis_x = quat.rotate(axis_x)
    new_axis_y = quat.rotate(axis_y)
    new_axis_z = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return 0, 0, 1
    elif angle < np.pi / 3 * 2 * 1.2:
        if abs(new_axis_x[2]) > abs(new_axis_y[2]):
            return 1, 0, 0
        else:
            return 0, 1, 0
        #else:
        #    raise Exception(f"Invalid axis {new_axis_x}")
    else:
        return 0, 0, -1


def get_approach_angle(model_quat, facing_direction):#get gripper approach angle
    if facing_direction == (0, 0, 1):
        return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = -1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()


if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")

    controller = ArmController()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    print("Waiting for detection of the models")
    rospy.sleep(0.5)
    legos = get_legos_pos(vision=True)
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

    for model_name, model_pose in legos:
    # TODO:
    #Implement the manipulation orchestration logic that consumes detected LEGO poses, 
    #computes appropriate grasp and approach actions, executes arm motion through the 
    #controller, and coordinates grasp attachment and release
    # You must name the service clients exactly as self.setstatic_srv, self.attach_srv, and self.detach_srv
    # END OF TODO

    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)

----------------------------


# LLM Response

[FILENAME: controller.py]
import math
import copy
import rclpy
from rclpy.node import Node
import numpy as np
import kinematics
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pyquaternion import Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

def get_controller_state(node, controller_topic, timeout_sec=None):
    # Create a temporary subscription to get the last message
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1
    )
    
    future = rclpy.Future()
    
    def state_callback(msg):
        if not future.done():
            future.set_result(msg)

    sub = node.create_subscription(
        JointTrajectoryControllerState,
        f"{controller_topic}/state",
        state_callback,
        qos_profile
    )

    node.get_logger().info(f"Waiting for controller state on {controller_topic}/state...")
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    node.destroy_subscription(sub)
    
    if future.done():
        return future.result()
    else:
        node.get_logger().warn(f"Timeout waiting for controller state on {controller_topic}/state")
        return None


class ArmController(Node):
    def __init__(self, gripper_state=0, controller_topic="/trajectory_controller"):
        super().__init__('arm_controller')
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.gripper_state = gripper_state

        self.controller_topic = controller_topic
        self.default_joint_trajectory = JointTrajectory()
        self.default_joint_trajectory.joint_names = self.joint_names

        # Wait for the controller state to be available
        joint_states_msg = get_controller_state(self, controller_topic, timeout_sec=10)
        if joint_states_msg:
            joint_states = joint_states_msg.actual.positions
            x, y, z, rot = kinematics.get_pose(joint_states)
            self.gripper_pose = (x, y, z), Quaternion(matrix=rot)
        else:
            self.get_logger().error("Failed to get initial joint states. Setting default gripper pose.")
            self.gripper_pose = (0.0, 0.0, 0.0), Quaternion(1, 0, 0, 0) # Default/fallback pose

        # Create a publisher for the joint trajectory
        self.joints_pub = self.create_publisher(
            JointTrajectory,
            f"{self.controller_topic}/command",
            10)
        
        self.get_logger().info("ArmController initialized.")

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        """
    	Execute an end-effector motion to the target pose by generating
    	and publishing joint trajectories, and update the internal arm state
    	based on execution feedback.
        """
        current_x, current_y, current_z = self.gripper_pose[0]
        current_quat = self.gripper_pose[1]

        target_x = x if x is not None else current_x
        target_y = y if y is not None else current_y
        target_z = z if z is not None else current_z
        target_quat = target_quat if target_quat is not None else current_quat

        # If z_raise is specified, first move up, then to target x,y, then down
        if z_raise > 0:
            # Move up to z_raise above current z
            self.get_logger().info(f"Moving up to z={current_z + z_raise}")
            self.send_joints(*kinematics.get_joints(current_x, current_y, current_z + z_raise, current_quat.rotation_matrix))
            if blocking:
                self.wait_for_position()
            
            # Move to target x, y at the raised z
            self.get_logger().info(f"Moving to x={target_x}, y={target_y} at z={current_z + z_raise}")
            self.send_joints(*kinematics.get_joints(target_x, target_y, current_z + z_raise, target_quat.rotation_matrix))
            if blocking:
                self.wait_for_position()
            
            # Move down to target z
            self.get_logger().info(f"Moving down to z={target_z}")
            self.send_joints(*kinematics.get_joints(target_x, target_y, target_z, target_quat.rotation_matrix))
            if blocking:
                self.wait_for_position()
        else:
            # Direct move to target pose
            self.get_logger().info(f"Moving to x={target_x}, y={target_y}, z={target_z}")
            self.send_joints(*kinematics.get_joints(target_x, target_y, target_z, target_quat.rotation_matrix))
            if blocking:
                self.wait_for_position()

        # Update internal gripper pose after movement
        self.gripper_pose = (target_x, target_y, target_z), target_quat


    def send_joints(self, joint_states, duration=1.0):  # x,y,z and orientation of lego block
        traj = copy.deepcopy(self.default_joint_trajectory)

        pts = JointTrajectoryPoint()
        pts.positions = joint_states
        pts.velocities = [0.0] * len(joint_states)
        pts.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        # Set the points to the trajectory
        traj.points = [pts]
        # Publish the message
        self.joints_pub.publish(traj)

    def wait_for_position(self, timeout_sec=2, tol_pos=0.01, tol_vel=0.01):
        self.get_logger().info("Waiting for arm to reach position...")
        start_time = self.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=timeout_sec)

        while self.get_clock().now() < end_time:
            msg = get_controller_state(self, self.controller_topic, timeout_sec=1)
            if msg is None:
                self.get_logger().warn("Failed to get controller state while waiting for position.")
                continue

            v_sum = np.sum(np.abs(msg.actual.velocities))
            
            if v_sum < tol_vel:
                all_positions_reached = True
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        all_positions_reached = False
                        break
                if all_positions_reached:
                    self.get_logger().info("Arm reached target position.")
                    return
            
            rclpy.spin_once(self, timeout_sec=0.1) # Process callbacks

        self.get_logger().warn("Timeout waiting for position")

[FILENAME: motion_planning.py]
#!/usr/bin/python3

import os
import math
import copy
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from control_msgs.msg import JointTolerance
from controller import ArmController
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, Attach

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "X1-Y2-Z1": {
        "home": [0.264589, -0.293903, 0.777] 
    },
    "X2-Y2-Z2": {
        "home": [0.277866, -0.724482, 0.777] 
    },
    "X1-Y3-Z2": {
        "home": [0.268053, -0.513924, 0.777]  
    },
    "X1-Y2-Z2": {
        "home": [0.429198, -0.293903, 0.777] 
    },
    "X1-Y2-Z2-CHAMFER": {
        "home": [0.592619, -0.293903, 0.777]  
    },
    "X1-Y4-Z2": {
        "home": [0.108812, -0.716057, 0.777] 
    },
    "X1-Y1-Z2": {
        "home": [0.088808, -0.295820, 0.777] 
    },
    "X1-Y2-Z2-TWINFILLET": {
        "home": [0.103547, -0.501132, 0.777] 
    },
    "X1-Y3-Z2-FILLET": {
        "home": [0.433739, -0.507130, 0.777]  
    },
    "X1-Y4-Z1": {
        "home": [0.589908, -0.501033, 0.777]  
    },
    "X2-Y2-Z2-FILLET": {
        "home": [0.442505, -0.727271, 0.777] 
    }
}

for model, model_info in MODELS_INFO.items():
    pass
    #MODELS_INFO[model]["home"] = model_info["home"] + np.array([0.0, 0.10, 0.0])

for model, info in MODELS_INFO.items():
    model_json_path = os.path.join(PKG_PATH, "..", "models", f"lego_{model}", "model.json")
    # make path absolute
    model_json_path = os.path.abspath(model_json_path)
    # check path exists
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = (np.max(corners[:, 0]) - np.min(corners[:, 0]))
    size_y = (np.max(corners[:, 1]) - np.min(corners[:, 1]))
    size_z = (np.max(corners[:, 2]) - np.min(corners[:, 2]))

    #print(f"{model}: {size_x:.3f} x {size_y:.3f} x {size_z:.3f}")

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10.0 # ROS2 JointTolerance velocity is float64

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planning_node')
        self.controller = ArmController()

        self.action_gripper = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.get_logger().info("Waiting for gripper action server...")
        self.action_gripper.wait_for_server()
        self.get_logger().info("Gripper action server found.")

        self.setstatic_srv = self.create_client(SetStatic, "/link_attacher_node/setstatic")
        self.attach_srv = self.create_client(Attach, "/link_attacher_node/attach")
        self.detach_srv = self.create_client(Attach, "/link_attacher_node/detach")

        self.get_logger().info("Waiting for link attacher services...")
        self.setstatic_srv.wait_for_service()
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()
        self.get_logger().info("Link attacher services found.")

    def get_gazebo_model_name(self, model_name, vision_model_pose):
        """
            Get the name of the model inside gazebo. It is needed for link attacher plugin.
        """
        self.get_logger().info("Waiting for /gazebo/model_states...")
        models = self.wait_for_message("/gazebo/model_states", ModelStates, timeout_sec=10)
        if models is None:
            self.get_logger().error("Failed to get Gazebo model states.")
            return None
        
        epsilon = 0.05
        for gazebo_model_name, model_pose in zip(models.name, models.pose):
            if model_name not in gazebo_model_name:
                continue
            # Get everything inside a square of side epsilon centered in vision_model_pose
            ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
            if ds <= epsilon:
                return gazebo_model_name
        raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")

    def get_model_name(self, gazebo_model_name):
        return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]

    def get_legos_pos(self, vision=False):
        #get legos position reading vision topic
        if vision:
            self.get_logger().info("Waiting for /lego_detections...")
            legos_msg = self.wait_for_message("/lego_detections", ModelStates, timeout_sec=10)
            if legos_msg is None:
                self.get_logger().error("Failed to get lego detections.")
                return []
            legos = legos_msg
        else:
            self.get_logger().info("Waiting for /gazebo/model_states...")
            models_msg = self.wait_for_message("/gazebo/model_states", ModelStates, timeout_sec=10)
            if models_msg is None:
                self.get_logger().error("Failed to get Gazebo model states.")
                return []
            
            legos = ModelStates()
            for name, pose in zip(models_msg.name, models_msg.pose):
                if "X" not in name:
                    continue
                name_stripped = self.get_model_name(name)

                legos.name.append(name_stripped)
                legos.pose.append(pose)
        return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]

    def straighten(self, model_pose, gazebo_model_name):
        x = model_pose.position.x
        y = model_pose.position.y
        z = model_pose.position.z
        model_quat = PyQuaternion(
            x=model_pose.orientation.x,
            y=model_pose.orientation.y,
            z=model_pose.orientation.z,
            w=model_pose.orientation.w)

        model_size = MODELS_INFO[self.get_model_name(gazebo_model_name)]["size"]

        """
            Calculate approach quaternion and target quaternion
        """

        facing_direction = get_axis_facing_camera(model_quat)
        approach_angle = get_approach_angle(model_quat, facing_direction)

        self.get_logger().info(f"Lego is facing {facing_direction}")
        self.get_logger().info(f"Angle of approaching measures {approach_angle:.2f} deg")

        # Calculate approach quat
        approach_quat = get_approach_quat(facing_direction, approach_angle)

        # Get above the object
        self.controller.move_to(x, y, target_quat=approach_quat)

        # Calculate target quat
        regrip_quat = DEFAULT_QUAT
        if facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):  # Side
            target_quat = DEFAULT_QUAT
            pitch_angle = -math.pi/2 + 0.2

            if abs(approach_angle) < math.pi/2:
                target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
            else:
                target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
            target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat

            if facing_direction == (0, 1, 0):
                regrip_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) * regrip_quat

        elif facing_direction == (0, 0, -1):
            """
                Pre-positioning
            """
            self.controller.move_to(z=z, target_quat=approach_quat)
            self.close_gripper(gazebo_model_name, model_size[0])

            tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2*math.pi/6) * DEFAULT_QUAT
            self.controller.move_to(SAFE_X, SAFE_Y, z+0.05, target_quat=tmp_quat, z_raise=0.1)  # Move to safe position
            self.controller.move_to(z=z)
            self.open_gripper(gazebo_model_name)

            approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)

            target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)  # Add a yaw rotation of 180 deg

            regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
        else:
            target_quat = DEFAULT_QUAT
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

        """
            Grip the model
        """
        if facing_direction == (0, 0, 1) or facing_direction == (0, 0, -1):
            closure = model_size[0]
            z = SURFACE_Z + model_size[2] / 2
        elif facing_direction == (1, 0, 0):
            closure = model_size[1]
            z = SURFACE_Z + model_size[0] / 2
        elif facing_direction == (0, 1, 0):
            closure = model_size[0]
            z = SURFACE_Z + model_size[1] / 2
        self.controller.move_to(z=z, target_quat=approach_quat)
        self.close_gripper(gazebo_model_name, closure)

        """
            Straighten model if needed
        """
        if facing_direction != (0, 0, 1):
            z = SURFACE_Z + model_size[2]/2

            self.controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
            self.controller.move(dz=-0.05)
            self.open_gripper(gazebo_model_name)

            # Re grip the model
            self.controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
            self.close_gripper(gazebo_model_name, model_size[0])


    def close_gripper(self, gazebo_model_name, closure=0):
        self.set_gripper(0.81-closure*10)
        self.get_logger().info("Gripper closing...")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        # Create dynamic joint
        if gazebo_model_name is not None:
            req = Attach.Request()
            req.model_name_1 = gazebo_model_name
            req.link_name_1 = "link"
            req.model_name_2 = "robot"
            req.link_name_2 = "wrist_3_link"
            self.attach_srv.call_async(req)
            self.get_logger().info(f"Attached {gazebo_model_name} to robot.")


    def open_gripper(self, gazebo_model_name=None):
        self.set_gripper(0.0)
        self.get_logger().info("Gripper opening...")

        # Destroy dynamic joint
        if gazebo_model_name is not None:
            req = Attach.Request()
            req.model_name_1 = gazebo_model_name
            req.link_name_1 = "link"
            req.model_name_2 = "robot"
            req.link_name_2 = "wrist_3_link"
            self.detach_srv.call_async(req)
            self.get_logger().info(f"Detached {gazebo_model_name} from robot.")


    def set_model_fixed(self, model_name):
        req_attach = Attach.Request()
        req_attach.model_name_1 = model_name
        req_attach.link_name_1 = "link"
        req_attach.model_name_2 = "ground_plane"
        req_attach.link_name_2 = "link"
        self.attach_srv.call_async(req_attach)

        req_setstatic = SetStatic.Request()
        self.get_logger().info(f"{model_name} TO HOME")
        req_setstatic.model_name = model_name
        req_setstatic.link_name = "link"
        req_setstatic.set_static = True

        self.setstatic_srv.call_async(req_setstatic)


    def get_approach_quat(self, facing_direction, approach_angle):
        quat = DEFAULT_QUAT
        if facing_direction == (0, 0, 1):
            pitch_angle = 0
            yaw_angle = 0
        elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
            pitch_angle = + 0.2
            if abs(approach_angle) < math.pi/2:
                yaw_angle = math.pi/2
            else:
                yaw_angle = -math.pi/2
        elif facing_direction == (0, 0, -1):
            pitch_angle = 0
            yaw_angle = 0
        else:
            raise ValueError(f"Invalid model state {facing_direction}")

        quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
        quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
        quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

        return quat


    def get_axis_facing_camera(self, quat):
        axis_x = np.array([1, 0, 0])
        axis_y = np.array([0, 1, 0])
        axis_z = np.array([0, 0, 1])
        new_axis_x = quat.rotate(axis_x)
        new_axis_y = quat.rotate(axis_y)
        new_axis_z = quat.rotate(axis_z)
        # get angle between new_axis and axis_z
        angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
        # get if model is facing up, down or sideways
        if angle < np.pi / 3:
            return 0, 0, 1
        elif angle < np.pi / 3 * 2 * 1.2:
            if abs(new_axis_x[2]) > abs(new_axis_y[2]):
                return 1, 0, 0
            else:
                return 0, 1, 0
            #else:
            #    raise Exception(f"Invalid axis {new_axis_x}")
        else:
            return 0, 0, -1


    def get_approach_angle(self, model_quat, facing_direction):#get gripper approach angle
        if facing_direction == (0, 0, 1):
            return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
        elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
            axis_x = np.array([0, 1, 0])
            axis_y = np.array([-1, 0, 0])
            new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
            # get angle between new_axis and axis_x
            dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
            det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
            return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
        elif facing_direction == (0, 0, -1):
            return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
        else:
            raise ValueError(f"Invalid model state {facing_direction}")


    def set_gripper(self, value):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = value  # From 0.0 to 0.8
        goal_msg.command.max_effort = -1.0  # Do not limit the effort

        self.get_logger().info(f"Sending gripper command: {value}")
        future = self.action_gripper.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected :(')
            return None

        self.get_logger().info('Gripper goal accepted :)')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result

    def wait_for_message(self, topic_name, msg_type, timeout_sec=None):
        self.get_logger().info(f"Waiting for message on {topic_name}...")
        future = rclpy.Future()
        
        def callback(msg):
            if not future.done():
                future.set_result(msg)

        # Use a transient local QoS profile to get the last published message
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        sub = self.create_subscription(msg_type, topic_name, callback, qos_profile)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        self.destroy_subscription(sub)
        
        if future.done():
            return future.result()
        else:
            self.get_logger().warn(f"Timeout waiting for message on {topic_name}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()

    node.controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    node.get_logger().info("Waiting for detection of the models")
    node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
    legos = node.get_legos_pos(vision=True)
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

    for model_name, model_pose in legos:
        node.get_logger().info(f"Processing model: {model_name} at ({model_pose.position.x:.2f}, {model_pose.position.y:.2f}, {model_pose.position.z:.2f})")
        
        # Get the actual Gazebo model name for link attachment
        gazebo_model_name = node.get_gazebo_model_name(model_name, model_pose)
        if gazebo_model_name is None:
            node.get_logger().error(f"Could not find Gazebo model for {model_name}. Skipping.")
            continue

        # 1. Approach the object
        target_x = model_pose.position.x
        target_y = model_pose.position.y
        target_z = model_pose.position.z + 0.1 # Approach from slightly above
        
        # Determine approach quaternion based on model orientation
        model_quat = PyQuaternion(
            x=model_pose.orientation.x, y=model_pose.orientation.y,
            z=model_pose.orientation.z, w=model_pose.orientation.w
        )
        facing_direction = node.get_axis_facing_camera(model_quat)
        approach_angle = node.get_approach_angle(model_quat, facing_direction)
        approach_quat = node.get_approach_quat(facing_direction, approach_angle)

        node.controller.move_to(target_x, target_y, target_z, target_quat=approach_quat, z_raise=0.05)
        
        # 2. Lower to grasp position
        model_size = MODELS_INFO[node.get_model_name(gazebo_model_name)]["size"]
        grasp_z = SURFACE_Z + model_size[2] / 2 + INTERLOCKING_OFFSET # Adjust for interlocking
        
        node.controller.move_to(target_x, target_y, grasp_z, target_quat=approach_quat)
        
        # 3. Grasp the object
        closure = model_size[0] # Assuming closure based on X dimension for simplicity
        node.close_gripper(gazebo_model_name, closure)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0)) # Allow gripper to close and attachment to happen

        # 4. Lift the object
        node.controller.move_to(target_x, target_y, target_z + 0.1, target_quat=approach_quat)

        # 5. Move to home position for this model type
        home_pos = MODELS_INFO[node.get_model_name(gazebo_model_name)]["home"]
        node.controller.move_to(home_pos[0], home_pos[1], home_pos[2] + 0.1, target_quat=DEFAULT_QUAT, z_raise=0.1)
        node.controller.move_to(home_pos[0], home_pos[1], home_pos[2], target_quat=DEFAULT_QUAT)

        # 6. Release the object
        node.open_gripper(gazebo_model_name)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0)) # Allow gripper to open and detachment to happen

        # 7. Set the model fixed on the ground plane
        node.set_model_fixed(gazebo_model_name)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

        # 8. Return to a safe position
        node.controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    node.get_logger().info("Moving to Default Position")
    node.controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    node.open_gripper()
    node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.4))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()