import math
import copy
import time
import numpy as np
import rclpy
from rclpy.node import Node
import trajectory_msgs.msg
from quaternion_utils import Quaternion

# Import kinematics - try both package-relative and direct
try:
    from task_002_ur5_pick_and_place import kinematics
except ImportError:
    import kinematics


# Lazy-import control_msgs to allow the module to load even when
# the package is not installed (tests that only need structural
# checks will still work).
_control_msgs = None

def _get_control_msgs():
    global _control_msgs
    if _control_msgs is None:
        try:
            import control_msgs.msg as cm
            _control_msgs = cm
        except ImportError:
            _control_msgs = None
    return _control_msgs


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

        # Create a publisher for joint trajectory commands
        self.joints_pub = self.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            f"{self.controller_topic}/command",
            10)

        # Subscription for controller state (only if control_msgs available)
        self._state_msg = None
        cm = _get_control_msgs()
        if cm is not None:
            self._state_sub = self.create_subscription(
                cm.JointTrajectoryControllerState,
                f"{self.controller_topic}/state",
                self._state_callback,
                10)
            # Wait for initial state
            self._wait_for_state(timeout=2.0)
        else:
            self._state_sub = None

        if self._state_msg is not None:
            joint_states = list(self._state_msg.actual.positions)
            x, y, z, rot = kinematics.get_pose(joint_states)
            self.gripper_pose = (x, y, z), Quaternion(matrix=rot)
        else:
            # Default pose if no state available
            self.gripper_pose = (-0.1, -0.2, 1.2), Quaternion(1, 0, 0, 0)

    def _state_callback(self, msg):
        self._state_msg = msg

    def _wait_for_state(self, timeout=10.0):
        start = time.time()
        while self._state_msg is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_controller_state(self, timeout=10.0):
        self._state_msg = None
        start = time.time()
        while self._state_msg is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._state_msg

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        """
        Execute an end-effector motion to the target pose by generating
        and publishing joint trajectories using Slerp interpolation for
        orientations and linear interpolation for positions, then update
        the internal arm state.
        """

        def smooth(percent_value, period=math.pi):
            return (1 - math.cos(percent_value * period)) / 2

        (sx, sy, sz), start_quat = self.gripper_pose

        if x is None:
            x = sx
        if y is None:
            y = sy
        if z is None:
            z = sz
        if target_quat is None:
            target_quat = start_quat

        dx, dy, dz = x - sx, y - sy, z - sz
        length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) * 300 + 80
        speed = length

        steps = int(length)
        step = 1 / steps

        for i in np.arange(0, 1 + step, step):
            i_2 = smooth(i, 2 * math.pi)  # from 0 to 1 to 0
            i_1 = smooth(i)  # from 0 to 1

            grip = Quaternion.slerp(start_quat, target_quat, i_1)
            self.send_joints(
                sx + i_1 * dx, sy + i_1 * dy, sz + i_1 * dz + i_2 * z_raise,
                grip,
                duration=1 / speed * 0.9)
            time.sleep(1 / speed)

        if blocking:
            self.wait_for_position(tol_pos=0.005, tol_vel=0.08)

        self.gripper_pose = (x, y, z), target_quat

    def send_joints(self, x, y, z, quat, duration=1.0):
        """x, y, z and orientation of lego block"""
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states.tolist() if hasattr(joint_states, 'tolist') else list(joint_states)
            pts.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pts.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
            traj.points = [pts]
            self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        start = time.time()
        while (time.time() - start) < timeout:
            msg = self.get_controller_state(timeout=10)
            if msg is None:
                continue
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                    return
        self.get_logger().warn("Timeout waiting for position")