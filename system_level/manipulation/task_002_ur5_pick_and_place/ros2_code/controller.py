import math
import copy
import rclpy
from rclpy.node import Node
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
from pyquaternion import Quaternion
from rclpy.duration import Duration
from rclpy.time import Time


def get_controller_state(node: Node, controller_topic, timeout=None):
    msg = node.create_subscription(
        control_msgs.msg.JointTrajectoryControllerState,
        f"{controller_topic}/state",
        lambda x: x,
        10)
    fut = rclpy.task.Future()

    def callback(msg):
        if not fut.done():
            fut.set_result(msg)

    sub = node.create_subscription(
        control_msgs.msg.JointTrajectoryControllerState,
        f"{controller_topic}/state",
        callback,
        10)

    try:
        return rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout).result()
    finally:
        node.destroy_subscription(sub)


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
        self.default_joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        self.default_joint_trajectory.joint_names = self.joint_names

        joint_states = get_controller_state(self, controller_topic).actual.positions
        x, y, z, rot = kinematics.get_pose(joint_states)
        self.gripper_pose = (x, y, z), Quaternion(matrix=rot)

        # Create a publisher for the joint trajectory
        self.joints_pub = self.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            f"{self.controller_topic}/command",
            10)

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
        if x is None or y is None or z is None or target_quat is None:
            self.get_logger().error("Target pose incomplete")
            return

        # Raise z by z_raise if specified
        z_target = z + z_raise

        # Send joints to raised position if z_raise > 0
        if z_raise > 0.0:
            self.send_joints(x, y, z_target, target_quat, duration=1.0)
            if blocking:
                self.wait_for_position(timeout=5)

        # Send joints to final position
        self.send_joints(x, y, z, target_quat, duration=2.0)
        if blocking:
            self.wait_for_position(timeout=5)

        # Update internal gripper pose state
        self.gripper_pose = (x, y, z), target_quat

    def send_joints(self, x, y, z, quat, duration=1.0):  # x,y,z and orientation of lego block
        # Solve for the joint angles, select the 5th solution
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states
            pts.velocities = [0, 0, 0, 0, 0, 0]
            pts.time_from_start = Duration(seconds=duration).to_msg()
            # Set the points to the trajectory
            traj.points = [pts]
            # Publish the message
            self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        end_time = self.get_clock().now() + Duration(seconds=timeout)
        while self.get_clock().now() < end_time:
            msg = get_controller_state(self, self.controller_topic, timeout=1.0)
            if msg is None:
                continue
            v = np.sum(np.abs(msg.actual.velocities))
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                else:
                    return
        self.get_logger().warn("Timeout waiting for position")
