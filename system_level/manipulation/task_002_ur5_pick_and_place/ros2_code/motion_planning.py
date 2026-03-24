import os
import math
import copy
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import SetStatic, Attach
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from controller import ArmController

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
    model_json_path = os.path.abspath(model_json_path)
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = (np.max(corners[:, 0]) - np.min(corners[:, 0]))
    size_y = (np.max(corners[:, 1]) - np.min(corners[:, 1]))
    size_z = (np.max(corners[:, 2]) - np.min(corners[:, 2]))

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = None  # Not used in ROS2 version here

def get_gazebo_model_name(node, model_name, vision_model_pose):
    models = node.create_subscription(ModelStates, "/gazebo/model_states", lambda msg: None, 10)
    # Instead of wait_for_message, use a temporary subscription and spin_once
    # But for simplicity, use node.get_logger and wait for message with future
    future = node.create_client(ModelStates, "/gazebo/model_states")
    # No service for ModelStates, so use wait_for_message helper:
    # We'll use rclpy.wait_for_message equivalent:
    from rclpy.task import Future
    from threading import Event

    msg_holder = {'msg': None}
    event = Event()

    def callback(msg):
        msg_holder['msg'] = msg
        event.set()

    sub = node.create_subscription(ModelStates, "/gazebo/model_states", callback, 10)
    event.wait(timeout=5.0)
    node.destroy_subscription(sub)
    models = msg_holder['msg']
    if models is None:
        raise RuntimeError("Timeout waiting for /gazebo/model_states")

    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")

def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]

def get_legos_pos(node, vision=False):
    if vision:
        msg_holder = {'msg': None}
        from threading import Event
        event = Event()
        def callback(msg):
            msg_holder['msg'] = msg
            event.set()
        sub = node.create_subscription(ModelStates, "/lego_detections", callback, 10)
        event.wait(timeout=5.0)
        node.destroy_subscription(sub)
        legos = msg_holder['msg']
        if legos is None:
            raise RuntimeError("Timeout waiting for /lego_detections")
    else:
        msg_holder = {'msg': None}
        from threading import Event
        event = Event()
        def callback(msg):
            msg_holder['msg'] = msg
            event.set()
        sub = node.create_subscription(ModelStates, "/gazebo/model_states", callback, 10)
        event.wait(timeout=5.0)
        node.destroy_subscription(sub)
        models = msg_holder['msg']
        if models is None:
            raise RuntimeError("Timeout waiting for /gazebo/model_states")
        legos = ModelStates()
        legos.name = []
        legos.pose = []
        for name, pose in zip(models.name, models.pose):
            if "X" not in name:
                continue
            name_short = get_model_name(name)
            legos.name.append(name_short)
            legos.pose.append(pose)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]

def straighten(controller, gazebo_model_name, model_pose):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)

    model_size = MODELS_INFO[get_model_name(gazebo_model_name)]["size"]

    facing_direction = get_axis_facing_camera(model_quat)
    approach_angle = get_approach_angle(model_quat, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")

    approach_quat = get_approach_quat(facing_direction, approach_angle)

    controller.move_to(x, y, target_quat=approach_quat)

    regrip_quat = DEFAULT_QUAT
    if facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
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
        controller.move_to(z=z, target_quat=approach_quat)
        close_gripper(gazebo_model_name, model_size[0])

        tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2*math.pi/6) * DEFAULT_QUAT
        controller.move_to(SAFE_X, SAFE_Y, z+0.05, target_quat=tmp_quat, z_raise=0.1)
        controller.move_to(z=z)
        open_gripper(gazebo_model_name)

        approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)

        target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)

        regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

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

    if facing_direction != (0, 0, 1):
        z = SURFACE_Z + model_size[2]/2

        controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
        controller.move(dz=-0.05)
        open_gripper(gazebo_model_name)

        controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
        close_gripper(gazebo_model_name, model_size[0])

def close_gripper(gazebo_model_name, closure=0):
    set_gripper(0.81-closure*10)
    rclpy.sleep(0.5)
    if gazebo_model_name is not None:
        req = Attach.Request()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_future = attach_client.call_async(req)
        rclpy.spin_until_future_complete(node, attach_future)

def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)
    if gazebo_model_name is not None:
        req = Attach.Request()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_future = detach_client.call_async(req)
        rclpy.spin_until_future_complete(node, detach_future)

def set_model_fixed(model_name):
    req_attach = Attach.Request()
    req_attach.model_name_1 = model_name
    req_attach.link_name_1 = "link"
    req_attach.model_name_2 = "ground_plane"
    req_attach.link_name_2 = "link"
    attach_future = attach_client.call_async(req_attach)
    rclpy.spin_until_future_complete(node, attach_future)

    req_setstatic = SetStatic.Request()
    print("{} TO HOME".format(model_name))
    req_setstatic.model_name = model_name
    req_setstatic.link_name = "link"
    req_setstatic.set_static = True
    setstatic_future = setstatic_client.call_async(req_setstatic)
    rclpy.spin_until_future_complete(node, setstatic_future)

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
    angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
    if angle < np.pi / 3:
        return 0, 0, 1
    elif angle < np.pi / 3 * 2 * 1.2:
        if abs(new_axis_x[2]) > abs(new_axis_y[2]):
            return 1, 0, 0
        else:
            return 0, 1, 0
    else:
        return 0, 0, -1

def get_approach_angle(model_quat, facing_direction):
    if facing_direction == (0, 0, 1):
        return model_quat.yaw_pitch_roll[0] - math.pi/2
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1]))
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0)
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0)
        return math.atan2(det, dot)
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

def set_gripper(value):
    goal_msg = GripperCommand.Goal()
    goal_msg.command.position = value
    goal_msg.command.max_effort = -1
    send_goal_future = action_gripper_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        raise RuntimeError("Gripper action goal rejected")
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result().result
    return result

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("send_joints")

    controller = ArmController()

    action_gripper_client = ActionClient(node, GripperCommand, "/gripper_controller/gripper_cmd")
    node.get_logger().info("Waiting for gripper action server...")
    action_gripper_client.wait_for_server()

    setstatic_client = node.create_client(SetStatic, "/link_attacher_node/setstatic")
    attach_client = node.create_client(Attach, "/link_attacher_node/attach")
    detach_client = node.create_client(Attach, "/link_attacher_node/detach")

    while not setstatic_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for setstatic service...")
    while not attach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for attach service...")
    while not detach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for detach service...")

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    node.get_logger().info("Waiting for detection of the models")
    rclpy.sleep(0.5)
    legos = get_legos_pos(node, vision=True)
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

    for model_name, model_pose in legos:
        gazebo_model_name = get_gazebo_model_name(node, model_name, model_pose)
        straighten(controller, gazebo_model_name, model_pose)
        set_model_fixed(gazebo_model_name)

    node.get_logger().info("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rclpy.sleep(0.4)

    node.destroy_node()
    rclpy.shutdown()
