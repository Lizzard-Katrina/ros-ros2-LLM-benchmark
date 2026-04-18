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
