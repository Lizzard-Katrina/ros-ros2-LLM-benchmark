#!/usr/bin/env python
import tf
import time
import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from ez_pick_and_place.srv import EzSceneSetupResponse
from moveit_msgs.msg import CollisionObject
from moveit_commander import MoveGroupCommander, PlanningScene

class EZToolSet:
    def __init__(self, node):
        self.node = node
        self.object_to_grasp = ""
        self.arm_move_group = None
        self.robot_commander = None
        self.gripper_move_group = ""
        self.arm_move_group_name = ""
        self.gripper_move_group_name = ""

        self.pose_factor = 1000

        self.tf2_buffer = None
        self.tf2_listener = None

        self.moveit_scene = None
        self.planning_srv = None
        self.add_model_srv = None
        self.load_model_srv = None

        self.ez_objects = dict()
        self.ez_obstacles = dict()

        self.pose_n_joint = dict()
        self.gripper_joint_bounds = dict()

        self.gripper_name = None
        self.gripper_frame = None

        self.target_place = None

        self.grasp_poses = []

        self.compute_ik_srv = None

        self.error_info = ""

        self.replanning = 0

        self.already_picked = False

        self.debug = False

    # Move the whole arm to the specified pose
    def move(self, pose):
        self.arm_move_group.set_pose_target(pose)
        return self.arm_move_group.go()

    # Move the whole arm to the specified state
    def moveToState(self, state):
        self.arm_move_group.set_joint_value_target(state)
        return self.arm_move_group.go()

    # Maximize all gripper joints
    def openGripper(self):
        curr_state = self.robot_commander.get_current_state()
        joint_pos = list(curr_state.joint_state.position)
        names = curr_state.joint_state.name
        for i in range(len(names)):
                if names[i] in self.gripper_joint_bounds:
                    joint_pos[i] = self.gripper_joint_bounds[names[i]]
        curr_state.joint_state.position = joint_pos
        return self.moveGripperToState(curr_state)

    # Move all joints based on a graspit result
    # and manipulate the scene object
    def grab(self, graspit_result):
        self.attachThis(self.object_to_grasp)
        res = self.moveGripper(graspit_result)
        self.detachThis(self.object_to_grasp)
        return res

    # Move all joints based on a graspit result
    def moveGripper(self, graspit_result):
        curr_state = self.robot_commander.get_current_state()
        joint_pos = list(curr_state.joint_state.position)
        names = curr_state.joint_state.name
        for i in range(len(graspit_result.joint_names)):
            for j in range(len(names)):
                if graspit_result.joint_names[i] == names[j]:
                    joint_pos[j] = self.gripper_joint_bounds[names[j]] - abs(graspit_result.points[0].positions[i] / self.pose_factor)
                    break
        curr_state.joint_state.position = joint_pos
        return self.moveGripperToState(curr_state)

    # Move all gripper joints to the specified state
    def moveGripperToState(self, state):
        self.gripper_move_group.set_joint_value_target(state)
        return self.gripper_move_group.go()

    # Shortcut of tf's lookup_transform
    def lookupTF(self, target_frame, source_frame):
        return self.tf2_buffer.lookup_transform(target_frame, source_frame, rclpy.Time(), rclpy.Duration(10))

    # Call graspit for the specified object
    def graspThis(self, object_name):
        target = CollisionObject()
        target.id = str(self.ez_objects[object_name][0])
        target.primitive_poses = [self.ez_objects[object_name][1].pose]
        response = self.planning_srv(group_name = self.gripper_name, target = target)
        return response.grasps

    # Shortcut of movegroup's attach_object
    def attachThis(self, object_name):
        touch_links = self.robot_commander.get_link_names(self.gripper_move_group_name)
        self.arm_move_group.attach_object(object_name, link_name=self.arm_move_group.get_end_effector_link(), touch_links=touch_links)

    # Shortcut of movegroup's detach_object
    def detachThis(self, object_name):
        self.arm_move_group.detach_object(object_name)

    # Pick and place!
    def uberPlan(self):
        return self.pick() and self.place()

    def pick(self):
        # Execute the reach motion based on filtered IK solutions.
        # Actuate the physical gripper closure.
        # SYSTEM COUPLING: You must manually update the Robot's URDF state in the 
        #    Planning Scene. Attach the target object to the gripper link so that 
        #    MoveIt treats the object as part of the robot's geometry for the next move.
        self.arm_move_group.set_pose_target(self.grasp_poses[0])
        self.arm_move_group.go()
        self.openGripper()
        self.attachThis(self.object_to_grasp)
        return True

    def place(self):
        # Query the 'Attached Object' pose from the current Scene state.
        # Calculate a safe drop-off trajectory that accounts for the payload's volume.
        # After release, Detach the object to return it to the 'Environment' state.
        target_pose, _ = self.calcTargetPose(self.ez_objects[self.object_to_grasp][1])
        self.arm_move_group.set_pose_target(target_pose)
        self.arm_move_group.go()
        self.detachThis(self.object_to_grasp)
        return True

    def getGripperBounds(self):
        for joint in self.gripper_move_group.get_joints():
            self.gripper_joint_bounds[joint] = self.robot_commander.get_joint(joint).max_bound()

    # Compute inverse kinematics for candidate poses
    # and discard those without a solution
    def discard(self, poses):
        validp = []
        validrs = []
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot_commander.get_current_state()
        req.ik_request.avoid_collisions = True
        for p in poses:
            req.ik_request.pose_stamped = p
            if self.debug:
                br = tf.TransformBroadcaster()
                br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z), (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w), rclpy.Time.now(), "candidate_grasp_pose", p.header.frame_id)
            k = self.compute_ik_srv(req)
            if k.error_code.val == 1:
                validp.append(p)
                validrs.append(k.solution)
        if validp:
            return [validp, validrs]
        return []

    # The planning service callback
    def startPlanning(self, req):
        # Initialize moveit stuff
        self.robot_commander = MoveGroupCommander("robot")
        self.arm_move_group = MoveGroupCommander(req.arm_move_group)
        self.gripper_move_group = MoveGroupCommander(req.gripper_move_group)

        # Save request values to use them later in the pipeline
        self.arm_move_group_name = req.arm_move_group
        self.object_to_grasp = req.graspit_target_object
        self.gripper_move_group_name = req.gripper_move_group
        self.target_place = req.target_place
        self.replanning = req.max_replanning if req.max_replanning > 0 else 0

        # Get bounds for each gipper joint, so we can later use the graspit values
        self.getGripperBounds()

        res = False
        while(self.replanning >= 0):
            self.error_info = ""
            if not self.already_picked:
                # Call graspit
                graspit_grasps = self.graspThis(req.graspit_target_object)

                # Generate grasp poses
                self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)

            res = self.uberPlan()
            self.replanning -= 1
            if res:
                break

        return EzStartPlanningResponse(success=res, error_info=self.error_info)

    # Graspit bodies are always referenced relatively to the "world" frame,
    # and units are not expressed in meters so translate the user's input
    def fixItForGraspIt(self, obj, pose_factor):
        for tryagain in range(0, 4):
            p = Pose()
            # If the user has provided the object wrt the world frame
            if obj.pose.header.frame_id == "world":
                p.position.x = obj.pose.pose.position.x * pose_factor
                p.position.y = obj.pose.pose.position.y * pose_factor
                p.position.z = obj.pose.pose.position.z * pose_factor
                p.orientation.x = obj.pose.pose.orientation.x
                p.orientation.y = obj.pose.pose.orientation.y
                p.orientation.z = obj.pose.pose.orientation.z
                p.orientation.w = obj.pose.pose.orientation.w
                return p
            # Else transform it to the world frame
            else:
                try:
                    transform = TransformStamped()
                    transform.header.stamp = rclpy.Time.now()
                    transform.header.frame_id = obj.pose.header.frame_id
                    transform.child_frame_id = "ez_fix_it_for_grasp_it"
                    transform.transform.translation.x = obj.pose.pose.position.x
                    transform.transform.translation.y = obj.pose.pose.position.y
                    transform.transform.translation.z = obj.pose.pose.position.z
                    transform.transform.rotation.x = obj.pose.pose.orientation.x
                    transform.transform.rotation.y = obj.pose.pose.orientation.y
                    transform.transform.rotation.z = obj.pose.pose.orientation.z
                    transform.transform.rotation.w = obj.pose.pose.orientation.w
                    self.tf2_buffer.set_transform(transform, "fixItForGraspIt")

                    trans = self.lookupTF("ez_fix_it_for_grasp_it", "world")

                    p.position.x = trans.transform.translation.x * pose_factor
                    p.position.y = trans.transform.translation.y * pose_factor
                    p.position.z = trans.transform.translation.z * pose_factor
                    p.orientation.x = trans.transform.rotation.x
                    p.orientation.y = trans.transform.rotation.y
                    p.orientation.z = trans.transform.rotation.z
                    p.orientation.w = trans.transform.rotation.w
                    return p
                except Exception as e:
                    print("fixItForGraspIt" + str(e))
        return None

    # GraspIt and MoveIt appear to have a 90 degree difference in the x axis (roll 90 degrees),
    # so translate everything for moveit compatibility
    def translateGraspIt2MoveIt(self, grasps, object_name):
        for tryagain in range(0,4):
            self.grasp_poses = []
            for g in grasps:
                try:
                    # World -> Object
                    transform = TransformStamped()
                    transform.header.stamp = rclpy.Time.now()
                    transform.header.frame_id = "world"
                    transform.child_frame_id = "target_object_frame"
                    transform.transform.translation.x = self.ez_objects[object_name][1].pose.position.x
                    transform.transform.translation.y = self.ez_objects[object_name][1].pose.position.y
                    transform.transform.translation.z = self.ez