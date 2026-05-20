import math
import copy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
from pyquaternion import Quaternion


def get_controller_state(node, controller_topic, timeout=None):
    msg = None
    def callback(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(
        control_msgs.msg.JointTrajectoryControllerState,
        f"{controller_topic}/state",
        callback,
        10
    )
    
    start_time = node.get_clock().now()
    while rclpy.ok() and msg is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if timeout is not None:
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                break
                
    node.destroy_subscription(sub)
    return msg


class ArmController:
    def __init__(self, node, gripper_state=0, controller_topic="/trajectory_controller"):
        self.node = node
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

        state_msg = get_controller_state(self.node, controller_topic)
        if state_msg is not None:
            joint_states = state_msg.actual.positions
            x, y, z, rot = kinematics.get_pose(joint_states)
            self.gripper_pose = (x, y, z), Quaternion(matrix=rot)
        else:
            self.gripper_pose = (0, 0, 0), Quaternion(1, 0, 0, 0)

        self.joints_pub = self.node.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            f"{self.controller_topic}/command",
            10
        )

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        (cx, cy, cz), cquat = self.gripper_pose
        tx = x if x is not None else cx
        ty = y if y is not None else cy
        tz = z if z is not None else cz
        tquat = target_quat if target_quat is not None else cquat

        if z_raise > 0.0:
            self.send_joints(cx, cy, cz + z_raise, cquat)
            if blocking:
                self.wait_for_position()
            self.send_joints(tx, ty, tz + z_raise, tquat)
            if blocking:
                self.wait_for_position()

        self.send_joints(tx, ty, tz, tquat)
        if blocking:
            self.wait_for_position()

        self.gripper_pose = ((tx, ty, tz), tquat)

    def send_joints(self, x, y, z, quat, duration=1.0):
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states
            pts.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pts.time_from_start = Duration(seconds=duration).to_msg()
            traj.points = [pts]
            self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2.0, tol_pos=0.01, tol_vel=0.01):
        start_time = self.node.get_clock().now()
        while rclpy.ok() and (self.node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            msg = get_controller_state(self.node, self.controller_topic, timeout=1.0)
            if msg is None:
                continue
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                else:
                    return
        self.node.get_logger().warn("Timeout waiting for position")