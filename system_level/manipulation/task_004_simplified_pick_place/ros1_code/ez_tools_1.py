#!/usr/bin/env python
import tf
import time
import numpy
import rospy
import moveit_commander

from math import sqrt, atan2
from tf.transformations import quaternion_from_euler, quaternion_multiply

from grasp_planning_graspit_msgs.srv import AddToDatabaseRequest, LoadDatabaseModelRequest
from moveit_msgs.srv import GetPositionIKRequest, GraspPlanning, GetPositionIK
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from ez_pick_and_place.srv import EzSceneSetupResponse
from moveit_msgs.msg import CollisionObject


class EZToolSet():

    # -----------------------------
    # State variables (unchanged)
    # -----------------------------
    object_to_grasp = ""
    arm_move_group = None
    robot_commander = None
    gripper_move_group = ""
    arm_move_group_name = ""
    gripper_move_group_name = ""

    pose_factor = 1000

    tf2_buffer = None
    tf2_listener = None

    moveit_scene = None
    planning_srv = None
    add_model_srv = None
    load_model_srv = None

    ez_objects = dict()
    ez_obstacles = dict()

    pose_n_joint = dict()
    gripper_joint_bounds = dict()

    gripper_name = None
    gripper_frame = None

    target_place = None
    grasp_poses = []

    compute_ik_srv = None

    error_info = ""
    replanning = 0
    already_picked = False
    debug = False

    # -----------------------------
    # Basic motion helpers
    # -----------------------------

    def move(self, pose):
        self.arm_move_group.set_pose_target(pose)
        return self.arm_move_group.go()

    def moveToState(self, state):
        self.arm_move_group.set_joint_value_target(state)
        return self.arm_move_group.go()

    def openGripper(self):
        curr_state = self.robot_commander.get_current_state()
        joint_pos = list(curr_state.joint_state.position)
        names = curr_state.joint_state.name
        for i in xrange(len(names)):
            if names[i] in self.gripper_joint_bounds:
                joint_pos[i] = self.gripper_joint_bounds[names[i]]
        curr_state.joint_state.position = joint_pos
        return self.moveGripperToState(curr_state)

    def moveGripperToState(self, state):
        self.gripper_move_group.set_joint_value_target(state)
        return self.gripper_move_group.go()

    # -----------------------------
    # IK filtering (Benchmark Task 004-1)
    # -----------------------------

    def discard(self, poses):

        valid_poses = []
        valid_robot_states = []

        req = GetPositionIKRequest()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot_commander.get_current_state()
        req.ik_request.avoid_collisions = True


        # TODO :

        #Compute inverse kinematics for candidate poses and
        #discard those without a valid IK solution.
        #END 

        if valid_poses:
            return [valid_poses, valid_robot_states]
        return []

    # -----------------------------
    # Pick pipeline (Benchmark Task 004-2)
    # -----------------------------

    def pick(self):
        """
        Open the gripper, move the arm to a valid grasp pose,
        and grasp the object.
        """

	if not self.already_picked:
            # GraspIt assumes maxed out joints, so that's what we do here
            self.openGripper()
            time.sleep(1)
            valid_g = self.discard(self.grasp_poses)

            if len(valid_g) > 0:
                for j in xrange(len(valid_g[0])):
                    self.arm_move_group.set_start_state_to_current_state()
                    if self.move(valid_g[0][j].pose):
                        time.sleep(1)
                        return self.grab(self.pose_n_joint[valid_g[0][j]])
                self.error_info = "Error while trying to pick the object!"
            else:
                self.error_info = "No valid grasps were found!"
            return False
        return True
    # -----------------------------
    # Place pipeline (Benchmark Task 004-3)
    # -----------------------------

    def place(self):
        # Attached objects are removed from the MoveIt scene
        # so we have to query before we attach it
        obj_trans = self.moveit_scene.get_object_poses([self.object_to_grasp])
        if not self.already_picked:
            self.attachThis(self.object_to_grasp)
            self.already_picked = True
        time.sleep(1)
        t, sol = self.calcTargetPose(obj_trans)
        if t and sol:
            if self.moveToState(sol) or self.move(t):
                time.sleep(1)
                self.openGripper()
                self.detachThis(self.object_to_grasp)
                return True
            self.error_info = "Error while trying to place the object!"
        else:
            self.error_info = "Error while trying to find a way to place the object!"
        return False
    # -----------------------------
    # Planning service entry point
    # (Benchmark Task 004-4)
    # -----------------------------

    def startPlanning(self, req):
        """
        Entry point of the pick-and-place pipeline.
        Initializes MoveIt, calls GraspIt, and executes pick & place.
        """

        # Initialize MoveIt interfaces
        self.robot_commander = moveit_commander.RobotCommander()
        self.arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)
        self.gripper_move_group = moveit_commander.MoveGroupCommander(req.gripper_move_group)

        self.arm_move_group_name = req.arm_move_group
        self.gripper_move_group_name = req.gripper_move_group
        self.object_to_grasp = req.graspit_target_object
        self.target_place = req.target_place
        self.replanning = req.max_replanning if req.max_replanning > 0 else 0

	self.getGripperBounds()
        res = False

        while self.replanning >= 0:
            self.error_info = ""

            if not self.already_picked:
                # Call GraspIt and generate grasp poses
                grasps = self.graspThis(req.graspit_target_object)
                self.translateGraspIt2MoveIt(grasps, req.graspit_target_object)

            res = self.pick() and self.place()
            self.replanning -= 1

            if res:
                break

        return res, self.error_info
