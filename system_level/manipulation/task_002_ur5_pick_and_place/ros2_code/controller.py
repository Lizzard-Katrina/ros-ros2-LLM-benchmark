import math
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
import numpy as np
import kinematics
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pyquaternion import Quaternion


class ArmController(Node):
    def __init__(self, gripper_state=0, controller_topic="/trajectory_controller"):
        super().__init__("arm_controller")
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

        self.joints_pub = self.create_publisher(
            JointTrajectory,
            f"{self.controller_topic}/command",
            10)

        self.get_controller_state_client = self.create_client(
            JointTrajectoryControllerState,
            f"{self.controller_topic}/state")

        while not self.get_controller_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.controller_topic}/state service not available, waiting...")

        req = JointTrajectoryControllerState()
        future = self.get_controller_state_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                else:
                    joint_states = response.actual.positions
                    x, y, z, rot = kinematics.get_pose(joint_states)
                    self.gripper_pose = (x, y, z), Quaternion(matrix=rot)
                    break

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        if x is None or y is None or z is None:
            x, y, z, _ = self.gripper_pose
        if target_quat is None:
            _, target_quat = self.gripper_pose

        joint_states = kinematics.get_joints(x, y, z, target_quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        pts = JointTrajectoryPoint()
        pts.positions = joint_states
        pts.velocities = [0, 0, 0, 0, 0, 0]
        pts.time_from_start = rclpy.time.Time(seconds=1.0, nanoseconds=0)
        traj.points = [pts]

        self.joints_pub.publish(traj)

        if blocking:
            end = self.get_logger().info("Waiting for position")
            while True:
                msg = self.get_controller_state_client.call(JointTrajectoryControllerState())
                v = np.sum(np.abs(msg.actual.velocities), axis=0)
                if v < 0.01:
                    for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                        if abs(actual - desired) > 0.01:
                            break
                    else:
                        break
                rclpy.spin_once(self)

    def send_joints(self, x, y, z, quat, duration=1.0):  
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        pts = JointTrajectoryPoint()
        pts.positions = joint_states
        pts.velocities = [0, 0, 0, 0, 0, 0]
        pts.time_from_start = rclpy.time.Time(seconds=duration, nanoseconds=0)
        traj.points = [pts]

        self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        end = self.get_logger().info("Waiting for position")
        while True:
            msg = self.get_controller_state_client.call(JointTrajectoryControllerState())
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                else:
                    break
            if (self.get_clock().now() - end).nanoseconds * 1e-9 > timeout:
                self.get_logger().warn("Timeout waiting for position")
                break
            rclpy.spin_once(self)