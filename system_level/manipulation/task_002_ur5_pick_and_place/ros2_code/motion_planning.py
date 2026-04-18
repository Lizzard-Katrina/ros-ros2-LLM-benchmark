#!/usr/bin/python3

import os
import math
import copy
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, Attach, Detach
from pyquaternion import Quaternion as PyQuaternion
import time

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

class MotionPlanning(Node):
    def __init__(self):
        super().__init__("motion_planning")
        self.controller = ArmController()
        self.action_gripper = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        while not self.action_gripper.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Gripper action server not available, waiting...")
        self.setstatic_srv = self.create_client(SetStatic, "/link_attacher_node/setstatic")
        self.attach_srv = self.create_client(Attach, "/link_attacher_node/attach")
        self.detach_srv = self.create_client(Detach, "/link_attacher_node/detach")
        while not self.setstatic_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/link_attacher_node/setstatic service not available, waiting...")
        while not self.attach_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/link_attacher_node/attach service not available, waiting...")
        while not self.detach_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/link_attacher_node/detach service not available, waiting...")

    def get_gazebo_model_name(self, model_name, vision_model_pose):
        models = self.get_logger().info("Waiting for model states")
        while True:
            try:
                models = rclpy.node.Node("get_model_states").get_logger().info("Getting model states")
                break
            except Exception as e:
                self.get_logger().info(f"Service call failed {e}")
                time.sleep(1)
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
        if vision:
            legos = self.get_logger().info("Waiting for lego detections")
            while True:
                try:
                    legos = rclpy.node.Node("get_lego_detections").get_logger().info("Getting lego detections")
                    break
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                    time.sleep(1)
        else:
            models = self.get_logger().info("Waiting for model states")
            while True:
                try:
                    models = rclpy.node.Node("get_model_states").get_logger().info("Getting model states")
                    break
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                    time.sleep(1)
            legos = []
            for name, pose in zip(models.name, models.pose):
                if "X" not in name:
                    continue
                name = self.get_model_name(name)
                legos.append((name, pose))
        return legos

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

        facing_direction = self.get_axis_facing_camera(model_quat)
        approach_angle = self.get_approach_angle(model_quat, facing_direction)

        print(f"Lego is facing {facing_direction}")
        print(f"Angle of approaching measures {approach_angle:.2f} deg")

        approach_quat = self.get_approach_quat(facing_direction, approach_angle)

        self.controller.move_to(x, y, target_quat=approach_quat)

        target_quat = self.get_target_quat(facing_direction, approach_angle, model_size)

        self.controller.move_to(x, y, z, target_quat)

        if facing_direction != (0, 0, 1):
            z = SURFACE_Z + model_size[2]/2

            self.controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
            self.controller.move(dz=-0.05)
            self.open_gripper(gazebo_model_name)

            self.controller.move_to(z=z, target_quat=target_quat, z_raise=0.1)
            self.close_gripper(gazebo_model_name, model_size[0])

    def close_gripper(self, gazebo_model_name, closure=0):
        goal = GripperCommand.Goal()
        goal.command.position = 0.81-closure*10
        goal.command.max_effort = -1
        self.action_gripper.send_goal_async(goal)
        self.action_gripper.wait_for_result()

        req = Attach.Request()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        self.attach_srv.call_async(req)

    def open_gripper(self, gazebo_model_name=None):
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = -1
        self.action_gripper.send_goal_async(goal)
        self.action_gripper.wait_for_result()

        if gazebo_model_name is not None:
            req = Detach.Request()
            req.model_name_1 = gazebo_model_name
            req.link_name_1 = "link"
            req.model_name_2 = "robot"
            req.link_name_2 = "wrist_3_link"
            self.detach_srv.call_async(req)

    def set_model_fixed(self, model_name):
        req = Attach.Request()
        req.model_name_1 = model_name
        req.link_name_1 = "link"
        req.model_name_2 = "ground_plane"
        req.link_name_2 = "link"
        self.attach_srv.call_async(req)

        req = SetStatic.Request()
        req.model_name = model_name
        req.link_name = "link"
        req.set_static = True
        self.setstatic_srv.call_async(req)

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

    def get_approach_angle(self, model_quat, facing_direction):
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

    def get_target_quat(self, facing_direction, approach_angle, model_size):
        if facing_direction == (0, 0, 1):
            target_quat = DEFAULT_QUAT
        elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
            target_quat = DEFAULT_QUAT
            pitch_angle = -math.pi/2 + 0.2
            target_quat = target_quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
        elif facing_direction == (0, 0, -1):
            target_quat = DEFAULT_QUAT
        else:
            raise ValueError(f"Invalid model state {facing_direction}")

        return target_quat

    def manipulate_legos(self):
        legos = self.get_legos_pos(vision=True)
        legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

        for model_name, model_pose in legos:
            gazebo_model_name = self.get_gazebo_model_name(model_name, model_pose)
            self.straighten(model_pose, gazebo_model_name)
            self.set_model_fixed(gazebo_model_name)

        self.get_logger().info("Moving to Default Position")
        self.controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
        self.open_gripper()
        time.sleep(0.4)

def main(args=None):
    rclpy.init(args=args)
    motion_planning = MotionPlanning()
    motion_planning.manipulate_legos()
    rclpy.spin(motion_planning)
    motion_planning.destroy_node()
    rclpy.shutdown()