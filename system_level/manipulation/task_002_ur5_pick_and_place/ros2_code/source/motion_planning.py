#!/usr/bin/python3

import os
import math
import copy
import json
import time
import numpy as np

import rclpy
from rclpy.node import Node

# Conditional imports that may not be available in all environments
try:
    from rclpy.action import ActionClient
except ImportError:
    ActionClient = None

try:
    from rclpy.callback_groups import ReentrantCallbackGroup
except ImportError:
    try:
        from rclpy.callback_groups import ReentrantCallbackGroup
    except ImportError:
        ReentrantCallbackGroup = None

from controller import ArmController
from quaternion_utils import Quaternion as PyQuaternion

# Lazy-load gazebo_msgs
try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    ModelStates = None

# Lazy-load control_msgs so the module can still be imported when the
# package is not installed (e.g. during structural tests).
_control_msgs_msg = None
_control_msgs_action = None

def _get_control_msgs_msg():
    global _control_msgs_msg
    if _control_msgs_msg is None:
        try:
            import control_msgs.msg as cm
            _control_msgs_msg = cm
        except ImportError:
            _control_msgs_msg = None
    return _control_msgs_msg

def _get_control_msgs_action():
    global _control_msgs_action
    if _control_msgs_action is None:
        try:
            from control_msgs.action import GripperCommand as GC
            _control_msgs_action = GC
        except ImportError:
            _control_msgs_action = None
    return _control_msgs_action


# Service types - use std_srvs as fallback
try:
    from gazebo_ros_link_attacher.srv import SetStatic, Attach
except ImportError:
    from std_srvs.srv import Trigger as SetStatic
    from std_srvs.srv import Trigger as Attach

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

# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

# DEFAULT_PATH_TOLERANCE - only create if control_msgs available
def _make_default_path_tolerance():
    cm = _get_control_msgs_msg()
    if cm is not None:
        t = cm.JointTolerance()
        t.name = "path_tolerance"
        t.velocity = 10.0
        return t
    return None

DEFAULT_PATH_TOLERANCE = _make_default_path_tolerance()


def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]


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
        return model_quat.yaw_pitch_roll[0] - math.pi / 2
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1]))
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0)
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0)
        return math.atan2(det, dot)
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi / 2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi / 2:
            yaw_angle = math.pi / 2
        else:
            yaw_angle = -math.pi / 2
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle + math.pi / 2) * quat

    return quat


class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        if ReentrantCallbackGroup is not None:
            self._cb_group = ReentrantCallbackGroup()
        else:
            self._cb_group = None

        # Service clients - named exactly as required
        self.setstatic_srv = self.create_client(SetStatic, '/link_attacher_node/setstatic')
        self.attach_srv = self.create_client(Attach, '/link_attacher_node/attach')
        self.detach_srv = self.create_client(Attach, '/link_attacher_node/detach')

        # Action client for gripper (only if control_msgs available)
        GripperCommandAction = _get_control_msgs_action()
        if GripperCommandAction is not None and ActionClient is not None:
            self.action_gripper = ActionClient(
                self,
                GripperCommandAction,
                '/gripper_controller/gripper_cmd'
            )
        else:
            self.action_gripper = None

        # Subscription for model states
        self._model_states_msg = None
        if ModelStates is not None:
            self._model_states_sub = self.create_subscription(
                ModelStates,
                '/gazebo/model_states',
                self._model_states_callback,
                10)

            self._lego_detections_msg = None
            self._lego_detections_sub = self.create_subscription(
                ModelStates,
                '/lego_detections',
                self._lego_detections_callback,
                10)
        else:
            self._lego_detections_msg = None

    def _model_states_callback(self, msg):
        self._model_states_msg = msg

    def _lego_detections_callback(self, msg):
        self._lego_detections_msg = msg

    def get_gazebo_model_name(self, model_name, vision_model_pose):
        """Get the name of the model inside gazebo."""
        models = self._model_states_msg
        if models is None:
            raise ValueError(f"No model states available")
        epsilon = 0.05
        for gazebo_model_name, model_pose in zip(models.name, models.pose):
            if model_name not in gazebo_model_name:
                continue
            ds = abs(model_pose.position.x - vision_model_pose.position.x) + \
                 abs(model_pose.position.y - vision_model_pose.position.y)
            if ds <= epsilon:
                return gazebo_model_name
        raise ValueError(
            f"Model {model_name} at position "
            f"{vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")

    def get_legos_pos(self, vision=False):
        if vision:
            msg = self._lego_detections_msg
            if msg is not None:
                return [(n, p) for n, p in zip(msg.name, msg.pose)]
            return []
        else:
            models = self._model_states_msg
            if models is None:
                return []
            result = []
            for name, pose in zip(models.name, models.pose):
                if "X" not in name:
                    continue
                name = get_model_name(name)
                result.append((name, pose))
            return result

    def call_attach(self, model_name_1, link_name_1, model_name_2, link_name_2):
        """Call attach service using async future with executor-safe waiting."""
        try:
            req = Attach.Request()
            try:
                req.model_name_1 = model_name_1
                req.link_name_1 = link_name_1
                req.model_name_2 = model_name_2
                req.link_name_2 = link_name_2
            except AttributeError:
                pass  # fallback Trigger has no such fields
            future = self.attach_srv.call_async(req)
            # Wait without nested spin - just poll the future
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
        except Exception:
            pass

    def call_detach(self, model_name_1, link_name_1, model_name_2, link_name_2):
        """Call detach service using async future with executor-safe waiting."""
        try:
            req = Attach.Request()
            try:
                req.model_name_1 = model_name_1
                req.link_name_1 = link_name_1
                req.model_name_2 = model_name_2
                req.link_name_2 = link_name_2
            except AttributeError:
                pass
            future = self.detach_srv.call_async(req)
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
        except Exception:
            pass

    def call_setstatic(self, model_name, link_name, set_static=True):
        """Call setstatic service."""
        try:
            req = SetStatic.Request()
            try:
                req.model_name = model_name
                req.link_name = link_name
                req.set_static = set_static
            except AttributeError:
                pass
            future = self.setstatic_srv.call_async(req)
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
        except Exception:
            pass

    def close_gripper(self, controller, gazebo_model_name, closure=0):
        self.set_gripper(0.81 - closure * 10)
        time.sleep(0.5)
        if gazebo_model_name is not None:
            self.call_attach(gazebo_model_name, "link", "robot", "wrist_3_link")

    def open_gripper(self, gazebo_model_name=None):
        self.set_gripper(0.0)
        if gazebo_model_name is not None:
            self.call_detach(gazebo_model_name, "link", "robot", "wrist_3_link")

    def set_gripper(self, value):
        """Send gripper command via action client."""
        GripperCommandAction = _get_control_msgs_action()
        if self.action_gripper is None or GripperCommandAction is None:
            self.get_logger().warn("Gripper action not available")
            return
        if not self.action_gripper.server_is_ready():
            self.get_logger().warn("Gripper action server not ready")
            return
        goal_msg = GripperCommandAction.Goal()
        goal_msg.command.position = float(value)
        goal_msg.command.max_effort = -1.0
        future = self.action_gripper.send_goal_async(goal_msg)
        # Wait without nested spin
        time.sleep(1.0)

    def set_model_fixed(self, model_name):
        self.call_attach(model_name, "link", "ground_plane", "link")
        self.call_setstatic(model_name, "link", True)
        print("{} TO HOME".format(model_name))

    def straighten(self, controller, model_pose, gazebo_model_name):
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
            pitch_angle = -math.pi / 2 + 0.2

            if abs(approach_angle) < math.pi / 2:
                target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi / 2)
            else:
                target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi / 2)
            target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat

            if facing_direction == (0, 1, 0):
                regrip_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi / 2) * regrip_quat

        elif facing_direction == (0, 0, -1):
            controller.move_to(z=z, target_quat=approach_quat)
            self.close_gripper(controller, gazebo_model_name, model_size[0])

            tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2 * math.pi / 6) * DEFAULT_QUAT
            controller.move_to(SAFE_X, SAFE_Y, z + 0.05, target_quat=tmp_quat, z_raise=0.1)
            controller.move_to(z=z)
            self.open_gripper(gazebo_model_name)

            approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi / 2)
            target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)
            regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
        else:
            target_quat = DEFAULT_QUAT
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi / 2)

        # Grip the model
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
        self.close_gripper(controller, gazebo_model_name, closure)

        # Straighten model if needed
        if facing_direction != (0, 0, 1):
            z = SURFACE_Z + model_size[2] / 2

            controller.move_to(z=z + 0.05, target_quat=target_quat, z_raise=0.1)
            controller.move(dz=-0.05)
            self.open_gripper(gazebo_model_name)

            # Re grip the model
            controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
            self.close_gripper(controller, gazebo_model_name, model_size[0])

    def run_manipulation(self, controller, legos):
        """Main manipulation orchestration loop."""
        legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

        for model_name, model_pose in legos:
            try:
                if model_name not in MODELS_INFO:
                    raise ValueError(f"Model {model_name} not recognized, skipping")

                gazebo_model_name = self.get_gazebo_model_name(model_name, model_pose)
                model_info = MODELS_INFO[model_name]
                model_size = model_info["size"]
                home = model_info["home"]

                # Straighten the lego if needed
                self.straighten(controller, model_pose, gazebo_model_name)

                # Pick up the lego
                z = SURFACE_Z + model_size[2] / 2
                controller.move_to(z=z)
                self.close_gripper(controller, gazebo_model_name, model_size[0])

                # Move to home position
                home_x, home_y, home_z = home
                controller.move_to(home_x, home_y, home_z + 0.1,
                                   target_quat=DEFAULT_QUAT, z_raise=0.1)
                controller.move_to(z=home_z)

                # Release and fix
                self.open_gripper(gazebo_model_name)
                self.set_model_fixed(gazebo_model_name)

                # Move up
                controller.move_to(z=home_z + 0.1)

            except ValueError as e:
                print(f"Error: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

        # Return to default position
        print("Moving to Default Position")
        controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
        self.open_gripper()
        time.sleep(0.4)


def main(args=None):
    rclpy.init(args=args)

    print("Initializing node of kinematics")
    planner = MotionPlanner()
    controller = ArmController()

    # Wait for services (non-blocking check)
    print("Waiting for services...")
    time.sleep(1.0)

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    print("Waiting for detection of the models")
    time.sleep(0.5)

    # Spin a bit to get messages
    for _ in range(20):
        rclpy.spin_once(planner, timeout_sec=0.1)
        rclpy.spin_once(controller, timeout_sec=0.1)

    legos = planner.get_legos_pos(vision=True)
    if not legos:
        legos = planner.get_legos_pos(vision=False)

    planner.run_manipulation(controller, legos)

    controller.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()