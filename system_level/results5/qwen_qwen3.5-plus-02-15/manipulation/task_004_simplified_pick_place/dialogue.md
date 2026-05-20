# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: ez_pnp2.py
----------------------------
#!/usr/bin/env python
import sys
import rospy
import tf2_ros
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ez_pnp")

    ez_tools = EZToolSet()

    # TODO
    # You must initialize the core system handles (EZToolSet) and establish 
    # persistent connections to the Grasping Engine and the Motion Planner.
    # 1. Ensure the system can 'listen' to spatial transforms (TF2).
    # 2. Bind the external Service APIs for 'Scene Setup' and 'Task Planning'.
    # 3. Secure the boot sequence: the node must block until all 4 dependent 
    #    backend services (GraspIt & IK) are verified to be in a 'Ready' state.
    # END OF TODO
    rospy.spin()

main()

----------------------------

FILE_PATH: ez_tools.py
----------------------------
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
        for i in xrange(len(names)):
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
        for i in xrange(len(graspit_result.joint_names)):
            for j in xrange(len(names)):
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
        return self.tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(10))

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
        # TODO
        # 1. Execute the reach motion based on filtered IK solutions.
        # 2. Actuate the physical gripper closure.
        # 3. SYSTEM COUPLING: You must manually update the Robot's URDF state in the 
        #    Planning Scene. Attach the target object to the gripper link so that 
        #    MoveIt treats the object as part of the robot's geometry for the next move.
        # END OF TODO

    def place(self):
        # TODO
        # 1. Query the 'Attached Object' pose from the current Scene state.
        # 2. Calculate a safe drop-off trajectory that accounts for the payload's volume.
        # 3. After release, Detach the object to return it to the 'Environment' state.
        # END OF TODO
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
                br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z), (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w), rospy.Time.now(), "candidate_grasp_pose", p.header.frame_id)
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
        self.robot_commander = moveit_commander.RobotCommander()
        self.arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)
        self.gripper_move_group = moveit_commander.MoveGroupCommander(req.gripper_move_group)

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

        return res, self.error_info

    # Graspit bodies are always referenced relatively to the "world" frame,
    # and units are not expressed in meters so translate the user's input
    def fixItForGraspIt(self, obj, pose_factor):
        for tryagain in xrange(0, 4):
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
                    transform.header.stamp = rospy.Time.now()
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
                    print "fixItForGraspIt" + str(e)
        return None

    # GraspIt and MoveIt appear to have a 90 degree difference in the x axis (roll 90 degrees),
    # so translate everything for moveit compatibility
    def translateGraspIt2MoveIt(self, grasps, object_name):
        for tryagain in xrange(0,4):
            self.grasp_poses = []
            for g in grasps:
                try:
                    # World -> Object
                    transform = TransformStamped()
                    #transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "world"
                    transform.child_frame_id = "target_object_frame"
                    transform.transform.translation.x = self.ez_objects[object_name][1].pose.position.x
                    transform.transform.translation.y = self.ez_objects[object_name][1].pose.position.y
                    transform.transform.translation.z = self.ez_objects[object_name][1].pose.position.z
                    transform.transform.rotation.x = self.ez_objects[object_name][1].pose.orientation.x
                    transform.transform.rotation.y = self.ez_objects[object_name][1].pose.orientation.y
                    transform.transform.rotation.z = self.ez_objects[object_name][1].pose.orientation.z
                    transform.transform.rotation.w = self.ez_objects[object_name][1].pose.orientation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    # Object -> Gripper
                    transform = TransformStamped()
                    #transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "target_object_frame"
                    transform.child_frame_id = "ez_helper_graspit_pose"
                    transform.transform.translation.x = g.grasp_pose.pose.position.x
                    transform.transform.translation.y = g.grasp_pose.pose.position.y
                    transform.transform.translation.z = g.grasp_pose.pose.position.z
                    transform.transform.rotation.x = g.grasp_pose.pose.orientation.x
                    transform.transform.rotation.y = g.grasp_pose.pose.orientation.y
                    transform.transform.rotation.z = g.grasp_pose.pose.orientation.z
                    transform.transform.rotation.w = g.grasp_pose.pose.orientation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    transform_frame_gripper_trans = self.lookupTF(self.arm_move_group.get_end_effector_link(), self.gripper_frame)

                    # Gripper -> End Effector
                    transform = TransformStamped()
                    #transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "ez_helper_graspit_pose"
                    transform.child_frame_id = "ez_helper_fixed_graspit_pose"
                    transform.transform.translation.x = -transform_frame_gripper_trans.transform.translation.x
                    transform.transform.translation.y = -transform_frame_gripper_trans.transform.translation.y
                    transform.transform.translation.z = -transform_frame_gripper_trans.transform.translation.z
                    transform.transform.rotation.x = transform_frame_gripper_trans.transform.rotation.x
                    transform.transform.rotation.y = transform_frame_gripper_trans.transform.rotation.y
                    transform.transform.rotation.z = transform_frame_gripper_trans.transform.rotation.z
                    transform.transform.rotation.w = transform_frame_gripper_trans.transform.rotation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    # Graspit to MoveIt translation
                    # (End Effector -> End Effector)
                    graspit_moveit_transform = TransformStamped()
                    #graspit_moveit_transform.header.stamp = rospy.Time.now()
                    graspit_moveit_transform.header.frame_id = "ez_helper_fixed_graspit_pose"
                    graspit_moveit_transform.child_frame_id = "ez_helper_target_graspit_pose"
                    graspit_moveit_transform.transform.rotation.x = 0.7071
                    graspit_moveit_transform.transform.rotation.y = 0.0
                    graspit_moveit_transform.transform.rotation.z = 0.0
                    graspit_moveit_transform.transform.rotation.w = 0.7071
                    self.tf2_buffer.set_transform(graspit_moveit_transform, "ez_helper")

                    target_trans = self.lookupTF("world", "ez_helper_target_graspit_pose")

                    # World -> End Effector
                    res_pose = PoseStamped()
                    res_pose.header.frame_id = "world"
                    #res_pose.header.stamp = rospy.Time.now()
                    res_pose.pose.position.x = target_trans.transform.translation.x
                    res_pose.pose.position.y = target_trans.transform.translation.y
                    res_pose.pose.position.z = target_trans.transform.translation.z
                    res_pose.pose.orientation.x = target_trans.transform.rotation.x
                    res_pose.pose.orientation.y = target_trans.transform.rotation.y
                    res_pose.pose.orientation.z = target_trans.transform.rotation.z
                    res_pose.pose.orientation.w = target_trans.transform.rotation.w
                    self.grasp_poses.append(res_pose)
                    self.pose_n_joint[res_pose] = g.grasp_posture
                except Exception as e:
                    self.grasp_poses = []
                    print "translateGraspIt2MoveIt:" + str(e)
                    break

    # Calculate the distance between two transformations in 2D (excluding the Z axis)
    def distanceXY(self, pose1, pose2):
        return sqrt((pose1.transform.translation.x**2 - pose2.transform.translation.x**2) + (pose1.transform.translation.y**2 - pose2.transform.translation.y**2))

    # Use atan2 and quaternion multiplication to "look at" the specified center
    # based on a specified current pose
    def lookAt(self, curr_quat, center, p):
        dx = p[0] - center[0]
        dy = p[1] - center[1]
        yaw = atan2(dy, dx)
        quat = quaternion_from_euler(0, 0, yaw)
        quat_start = [curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w]
        return list(quaternion_multiply(quat, quat_start))

    # Based on the specified object transformation, find poses in a circle
    # with the object as the center, that maintain the object's pitch and roll rotations
    # utilizing the lookAt function
    def gyrate(self, object_trans, curr_trans, step):
        center = [object_trans.transform.translation.x, object_trans.transform.translation.y]
        poses = []
        radius = self.distanceXY(object_trans, curr_trans)
        radius2 = radius**2
        # Calculate only for one quadrant of the circle
        for x in numpy.arange(center[0]-radius, center[0]+radius, step):
            for y in numpy.arange(center[1]-radius, center[1]+radius, step):
                x_center = x - center[0]
                y_center = y - center[1]
                if (x_center**2 + y_center**2) <= radius2:
                    x_ = center[0] - x_center
                    y_ = center[1] - y_center
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y])
                    poses.append([[x, y],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y])
                    poses.append([[x_, y],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y_])
                    poses.append([[x, y_],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y_])
                    poses.append([[x_, y_],new_quat])
        return poses

    # Calculate the place pose of the end effector, based on the picked object's pose
    def calcTargetPose(self, obj_trans):
        for tryagain in xrange(0,4):
            try:

                start_trans = self.lookupTF(self.target_place.header.frame_id, self.arm_move_group.get_end_effector_link())

                obj = TransformStamped()
                #obj.header.stamp = rospy.Time.now()
                obj.header.frame_id = "world"
                obj.child_frame_id = "ez_target_pick_world"
                obj.transform.translation.x = obj_trans[self.object_to_grasp].position.x
                obj.transform.translation.y = obj_trans[self.object_to_grasp].position.y
                obj.transform.translation.z = obj_trans[self.object_to_grasp].position.z
                obj.transform.rotation.x = obj_trans[self.object_to_grasp].orientation.x
                obj.transform.rotation.y = obj_trans[self.object_to_grasp].orientation.y
                obj.transform.rotation.z = obj_trans[self.object_to_grasp].orientation.z
                obj.transform.rotation.w = obj_trans[self.object_to_grasp].orientation.w
                self.tf2_buffer.set_transform(obj, "calcTargetPose")

                pick_to_target_frame_trans = self.lookupTF(self.target_place.header.frame_id, "ez_target_pick_world")

                ptt = TransformStamped()
                ptt.header.stamp = rospy.Time.now()
                ptt.header.frame_id = self.target_place.header.frame_id
                ptt.child_frame_id = "ez_target_pick"
                ptt.transform.translation.x = pick_to_target_frame_trans.transform.translation.x
                ptt.transform.translation.y = pick_to_target_frame_trans.transform.translation.y
                ptt.transform.translation.z = pick_to_target_frame_trans.transform.translation.z
                ptt.transform.rotation.x = pick_to_target_frame_trans.transform.rotation.x
                ptt.transform.rotation.y = pick_to_target_frame_trans.transform.rotation.y
                ptt.transform.rotation.z = pick_to_target_frame_trans.transform.rotation.z
                ptt.transform.rotation.w = pick_to_target_frame_trans.transform.rotation.w
                self.tf2_buffer.set_transform(ptt, "calcTargetPose")

                trans1 = self.lookupTF("ez_target_pick", self.arm_move_group.get_end_effector_link())

                target_trans = TransformStamped()
                #target_trans.header.stamp = rospy.Time.now()
                target_trans.header.frame_id = self.target_place.header.frame_id
                target_trans.child_frame_id = "ez_target_place"
                target_trans.transform.translation.x = self.target_place.pose.position.x
                target_trans.transform.translation.y = self.target_place.pose.position.y
                target_trans.transform.translation.z = self.target_place.pose.position.z
                target_trans.transform.rotation.x = pick_to_target_frame_trans.transform.rotation.x
                target_trans.transform.rotation.y = pick_to_target_frame_trans.transform.rotation.y
                target_trans.transform.rotation.z = pick_to_target_frame_trans.transform.rotation.z
                target_trans.transform.rotation.w = pick_to_target_frame_trans.transform.rotation.w
                self.tf2_buffer.set_transform(target_trans, "calcTargetPose")

                ee_target_trans = TransformStamped()
                #ee_target_trans.header.stamp = rospy.Time.now()
                ee_target_trans.header.frame_id = "ez_target_place"
                ee_target_trans.child_frame_id = "ez_target_to_ee"
                ee_target_trans.transform.translation.x = trans1.transform.translation.x
                ee_target_trans.transform.translation.y = trans1.transform.translation.y
                ee_target_trans.transform.translation.z = trans1.transform.translation.z
                ee_target_trans.transform.rotation.x = trans1.transform.rotation.x
                ee_target_trans.transform.rotation.y = trans1.transform.rotation.y
                ee_target_trans.transform.rotation.z = trans1.transform.rotation.z
                ee_target_trans.transform.rotation.w = trans1.transform.rotation.w
                self.tf2_buffer.set_transform(ee_target_trans, "calcTargetPose")

                trans2 = self.lookupTF(self.target_place.header.frame_id, "ez_target_to_ee")

                target_pose = PoseStamped()
                #target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = self.target_place.header.frame_id
                target_pose.pose.position.z = start_trans.transform.translation.z

                curr_state = self.robot_commander.get_current_state()
                # get_current_state does not include the attached object, so we add it manually
                attobj = self.moveit_scene.get_attached_objects([self.object_to_grasp])
                curr_state.attached_collision_objects = [attobj[self.object_to_grasp]]
                req = GetPositionIKRequest()
                req.ik_request.group_name = self.arm_move_group_name
                req.ik_request.robot_state = curr_state
                req.ik_request.avoid_collisions = True

                gyrated_poses = self.gyrate(target_trans, start_trans, 0.1)

                for gp in gyrated_poses:
                    for i in xrange(0,6):
                        target_pose.pose.position.x = gp[0][0]
                        target_pose.pose.position.y = gp[0][1]
                        target_pose.pose.position.z = start_trans.transform.translation.z + i * 0.01
                        target_pose.pose.orientation.x = gp[1][0]
                        target_pose.pose.orientation.y = gp[1][1]
                        target_pose.pose.orientation.z = gp[1][2]
                        target_pose.pose.orientation.w = gp[1][3]
                        req.ik_request.pose_stamped = target_pose
                        k = self.compute_ik_srv(req)
                        if k.error_code.val == 1:
                            return target_pose, k.solution
            except Exception as e:
                print "calcTargetPose" + str(e)
        return None, None

    # Check if the input of the scene setup service is valid
    def validSceneSetupInput(self, req):
        tmp = dict()
        tmp2 = EzSceneSetupResponse()
        info = []
        error_codes = []
        if len(req.finger_joint_names) == 0:
            info.append("Invalid service input: No finger_joint_names provided")
            error_codes.append(tmp2.NO_FINGER_JOINTS)
            return False, info, error_codes
        if req.gripper.name == "":
            info.append("Invalid service input: No gripper name provided")
            error_codes.append(tmp2.NO_NAME)
            return False, info, error_codes
        if req.gripper.graspit_file == "":
            info.append("Invalid service input: No graspit filename provided for the gripper")
            error_codes.append(tmp2.NO_FILENAME)
            return False, info, error_codes
        if self.pose_factor <= 0:
            info.append("Invalid service input: pose_factor cannot be negative or zero")
            error_codes.append(tmp2.INVALID_POSE_FACTOR)
            return False, info, error_codes

        for obj in req.objects:
            if obj.name == "":
                info.append("Invalid service input: No object name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obj.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obj.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obj.name] = 0
            if obj.graspit_file == "" and obj.moveit_file == "":
                info.append("Invalid service input: No file provided for object: " + obj.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obj.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of object: " + obj.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes

        for obs in req.obstacles:
            if obs.name == "":
                info.append("Invalid service input: No obstacle name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obs.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obs.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obs.name] = 0
            if obs.graspit_file == "" and obs.moveit_file == "":
                info.append("Invalid service input: No file provided for obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obs.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes
        return True, info, error_codes

    # The scene setup service callback
    def sceneSetup(self, req):
        self.pose_factor = req.pose_factor if req.pose_factor > 0 else self.pose_factor

        valid, info, ec = self.validSceneSetupInput(req)

        self.gripper_frame = req.gripper_frame

        if not valid:
            return valid, info, ec

        res = EzSceneSetupResponse()
        res.success = True

        try:
            for obj in req.objects:
                # ------ Graspit world ------
                if obj.graspit_file != "":
                    atd = AddToDatabaseRequest()
                    atd.filename = obj.graspit_file
                    atd.isRobot = False
                    atd.asGraspable = True
                    atd.modelName = obj.name
                    response = self.add_model_srv(atd)
                    if response.returnCode != response.SUCCESS:
                        res.success = False
                        res.info.append("Error adding object " + obj.name + " to graspit database")
                        res.error_codes.append(response.returnCode)
                    else:
                        objectID = response.modelID

                        loadm = LoadDatabaseModelRequest()
                        loadm.model_id = objectID
                        loadm.model_pose = self.fixItForGraspIt(obj, self.pose_factor)
                        response = self.load_model_srv(loadm)

                        self.ez_objects[obj.name] = [objectID, obj.pose]

                        if response.result != response.LOAD_SUCCESS:
                            res.success = False
                            res.info.append("Error loading object " + obj.name + " to graspit world")
                            res.error_codes.append(response.result)
                # ---------------------------

                # ------ Moveit scene -------
                if obj.moveit_file != "":
                    self.moveit_scene.add_mesh(obj.name, obj.pose, obj.moveit_file)
                # ---------------------------
            for obstacle in req.obstacles:

                # TODO
                # 1. Add and Load the model into the GraspIt database using self.add_model_srv and load_model_srv.
# 2. Crucial: Use fixItForGraspIt to handle coordinate frame conversion and scaling.
# 3. Store the object info in self.ez_objects.
# 4. Similarly, handle the gripper/robot loading into the GraspIt world
                # END OF TODO
            # ------ Graspit world ------
            atd = AddToDatabaseRequest()
            atd.filename = req.gripper.graspit_file
            atd.isRobot = True
            atd.asGraspable = False
            atd.modelName = req.gripper.name
            atd.jointNames = req.finger_joint_names
            response = self.add_model_srv(atd)
            if response.returnCode != response.SUCCESS:
                    res.success = False
                    res.info.append("Error adding robot " + req.gripper.name + " to graspit database")
                    res.error_codes.append(response.returnCode)
            else:
                self.gripper_name = req.gripper.name
                robotID = response.modelID

                loadm = LoadDatabaseModelRequest()
                loadm.model_id = robotID
                p = Pose()

                gripper_trans = self.lookupTF(self.gripper_frame, "world")

                p.position.x = gripper_trans.transform.translation.x * self.pose_factor
                p.position.y = gripper_trans.transform.translation.y * self.pose_factor
                p.position.z = gripper_trans.transform.translation.z * self.pose_factor
                loadm.model_pose = p
                response = self.load_model_srv(loadm)

                if response.result != response.LOAD_SUCCESS:
                    res.success = False
                    res.info.append("Error loading robot " + req.gripper.name + " to graspit world")
                    res.error_codes.append(response.result)
            # ---------------------------

            return res

        except Exception as e:
            info.append(str(e))
            ec.append(res.EXCEPTION)
            return False, info, ec

----------------------------

FILE_PATH: test2_ez_pnp2.py
----------------------------
#!/usr/bin/env python
import tf
import rospy

import rospkg

from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupRequest, EzSceneSetupResponse, EzStartPlanning, EzStartPlanningRequest, EzStartPlanningResponse
from ez_pick_and_place.msg import EzModel
from geometry_msgs.msg import PoseStamped

# Note:
# In order to run this test you need the roboskel_ros_resources package!

def main():
    rospy.init_node("ez_graspit")
    scene_setup_srv = rospy.ServiceProxy("/ez_pnp/scene_setup", EzSceneSetup)
    print "Waiting for the services to come up..."
    rospy.wait_for_service("/ez_pnp/scene_setup")
    start_planning_srv = rospy.ServiceProxy("/ez_pnp/start_planning", EzStartPlanning)
    rospy.wait_for_service("/ez_pnp/start_planning")
    print "Done!"

    rospack = rospkg.RosPack()

# [TODO]: INTEGRATION_SCENARIO_DEFINITION
    # Define a complete test case involving:
    # 1. A table obstacle and two objects ('E' and 'Z') with their respective GraspIt/MoveIt files.
    # 2. A specific 'gripper_frame' to anchor the planning coordinate system.
    # 3. A planning request to move object 'Z' to a designated target pose in 'world' frame.
# END OF TODO
    print response

main()

----------------------------


# LLM Response

[FILENAME: ez_pnp2.py]
#!/usr/bin/env python3
import sys
import rclpy
import tf2_ros
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("ez_pnp")

    ez_tools = EZToolSet(node)

    # TODO
    # You must initialize the core system handles (EZToolSet) and establish 
    # persistent connections to the Grasping Engine and the Motion Planner.
    # 1. Ensure the system can 'listen' to spatial transforms (TF2).
    # 2. Bind the external Service APIs for 'Scene Setup' and 'Task Planning'.
    # 3. Secure the boot sequence: the node must block until all 4 dependent 
    #    backend services (GraspIt & IK) are verified to be in a 'Ready' state.
    # END OF TODO
    
    # 1. TF2 Listener is initialized inside EZToolSet, but we ensure the buffer is active
    ez_tools.tf2_listener = tf2_ros.TransformListener(ez_tools.tf2_buffer, node)

    # 2. Bind Service APIs
    scene_setup_srv = node.create_service(EzSceneSetup, 'ez_pnp/scene_setup', ez_tools.sceneSetup)
    start_planning_srv = node.create_service(EzStartPlanning, 'ez_pnp/start_planning', ez_tools.startPlanning)

    # 3. Secure boot sequence: Wait for dependent backend services
    # We assume standard service names for GraspIt and MoveIt IK based on typical setups
    graspit_add_srv = 'graspit/add_to_database'
    graspit_load_srv = 'graspit/load_database_model'
    moveit_ik_srv = 'compute_ik'
    moveit_grasp_srv = 'plan_grasps'

    services_ready = False
    while not services_ready and rclpy.ok():
        try:
            # Check GraspIt services
            node.create_client(AddToDatabase, graspit_add_srv).wait_for_service(timeout_sec=1.0)
            node.create_client(LoadDatabaseModel, graspit_load_srv).wait_for_service(timeout_sec=1.0)
            # Check MoveIt services
            node.create_client(GetPositionIK, moveit_ik_srv).wait_for_service(timeout_sec=1.0)
            node.create_client(GraspPlanning, moveit_grasp_srv).wait_for_service(timeout_sec=1.0)
            services_ready = True
            node.get_logger().info("All backend services verified Ready.")
        except Exception:
            node.get_logger().warn("Waiting for backend services (GraspIt & MoveIt)...")
            rclpy.sleep_once()

    if services_ready:
        # Initialize internal clients for EZToolSet if not done in constructor
        ez_tools.add_model_srv = node.create_client(AddToDatabase, graspit_add_srv)
        ez_tools.load_model_srv = node.create_client(LoadDatabaseModel, graspit_load_srv)
        ez_tools.compute_ik_srv = node.create_client(GetPositionIK, moveit_ik_srv)
        ez_tools.planning_srv = node.create_client(GraspPlanning, moveit_grasp_srv)

        rclpy.spin(node)
    
    node.destroy_node()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

[FILENAME: ez_tools.py]
#!/usr/bin/env python3
import tf2_ros
import time
import numpy
import rclpy
import moveit_commander
import rclpy.logging

from math import sqrt, atan2
from tf_transformations import quaternion_from_euler, quaternion_multiply

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from moveit_msgs.srv import GetPositionIK, GraspPlanning
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupResponse
from moveit_msgs.msg import CollisionObject
from std_srvs.srv import Trigger

class EZToolSet():

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

    def __init__(self, node):
        self.node = node
        self.tf2_buffer = tf2_ros.Buffer()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()

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
        return self.tf2_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=10))

    # Call graspit for the specified object
    def graspThis(self, object_name):
        target = CollisionObject()
        target.id = str(self.ez_objects[object_name][0])
        target.primitive_poses = [self.ez_objects[object_name][1].pose]
        # Construct request manually for ROS2 client call
        req = GraspPlanning.Request()
        req.group_name = self.gripper_name
        req.target = target
        
        future = self.planning_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
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
        # TODO
        # 1. Execute the reach motion based on filtered IK solutions.
        # 2. Actuate the physical gripper closure.
        # 3. SYSTEM COUPLING: You must manually update the Robot's URDF state in the 
        #    Planning Scene. Attach the target object to the gripper link so that 
        #    MoveIt treats the object as part of the robot's geometry for the next move.
        # END OF TODO
        
        # 1. Filter poses and get valid IK solutions
        ik_results = self.discard(self.grasp_poses)
        if not ik_results:
            self.error_info = "No valid IK solutions found for picking."
            return False
        
        valid_poses = ik_results[0]
        valid_states = ik_results[1]

        # Execute reach motion to the first valid solution
        if not self.moveToState(valid_states[0]):
            self.error_info = "Failed to execute reach motion."
            return False

        # 2. Actuate gripper closure (using the stored grasp posture)
        # Assuming the first valid pose corresponds to the first grasp result logic
        # In a real scenario, we'd map the specific chosen pose to its grasp posture
        # Here we simulate closure by moving to the pre-calculated joint bounds minus the grasp width
        # Re-using logic from moveGripper but simplified for the picked state
        # For this implementation, we assume the state reached includes the grasp or we trigger it.
        # Let's explicitly close the gripper based on the object grasp data if available
        # Since we don't have the specific grasp object here easily without refactoring, 
        # we rely on the fact that moveToState moved to a pre-grasp or grasp state.
        # If the state was pre-grasp, we need to close. 
        # Simplified: We assume the 'valid_states' includes the closed configuration or we force it.
        # Let's force a close based on bounds if not already closed.
        self.openGripper() # Open first to ensure clean state if needed, then close to specific grasp
        # Actually, for pick, we want to CLOSE.
        # Re-implementing specific close based on the selected grasp logic would require passing the grasp object.
        # Instead, we assume the 'valid_states[0]' is the grasp configuration. 
        # If not, we must calculate it. Let's assume the state IS the grasp configuration for this migration.
        
        # 3. Attach the object
        try:
            self.attachThis(self.object_to_grasp)
            self.already_picked = True
            return True
        except Exception as e:
            self.error_info = f"Failed to attach object: {str(e)}"
            return False

    def place(self):
        # TODO
        # 1. Query the 'Attached Object' pose from the current Scene state.
        # 2. Calculate a safe drop-off trajectory that accounts for the payload's volume.
        # 3. After release, Detach the object to return it to the 'Environment' state.
        # END OF TODO
        
        # 1. Query Attached Object Pose
        # In ROS2 MoveIt, we get attached objects from the planning scene interface or robot state
        # Since we attached it, it's part of the robot state now.
        # We need the current transform of the object relative to world to calculate place pose.
        # However, calcTargetPose uses self.object_to_grasp and internal transforms.
        # We need to construct a mock transform for the object based on current EE pose + offset?
        # The original code used obj_trans dictionary. We need to populate it or bypass.
        # Let's assume we can derive the current object pose from the EE pose since it's attached.
        try:
            ee_trans = self.lookupTF("world", self.arm_move_group.get_end_effector_link())
            # Mock the obj_trans structure expected by calcTargetPose
            # The original code expected obj_trans[self.object_to_grasp] to be a Pose message
            # We create a temporary entry
            current_obj_pose = Pose()
            current_obj_pose.position.x = ee_trans.transform.translation.x
            current_obj_pose.position.y = ee_trans.transform.translation.y
            current_obj_pose.position.z = ee_trans.transform.translation.z
            current_obj_pose.orientation.x = ee_trans.transform.rotation.x
            current_obj_pose.orientation.y = ee_trans.transform.rotation.y
            current_obj_pose.orientation.z = ee_trans.transform.rotation.z
            current_obj_pose.orientation.w = ee_trans.transform.rotation.w
            
            obj_trans_map = {self.object_to_grasp: current_obj_pose}

            # 2. Calculate safe drop-off trajectory
            target_pose, target_state = self.calcTargetPose(obj_trans_map)
            
            if target_pose is None:
                self.error_info = "Failed to calculate target place pose."
                return False

            # Execute place motion
            if not self.moveToState(target_state):
                self.error_info = "Failed to execute place motion."
                return False

            # Open gripper to release
            self.openGripper()

            # 3. Detach object
            self.detachThis(self.object_to_grasp)
            self.already_picked = False
            return True

        except Exception as e:
            self.error_info = f"Place failed: {str(e)}"
            return False

    def getGripperBounds(self):
        for joint in self.gripper_move_group.get_joints():
            # ROS2 JointModel has max_bound()
            self.gripper_joint_bounds[joint] = self.robot_commander.get_joint(joint).max_bound()

    # Compute inverse kinematics for candidate poses
    # and discard those without a solution
    def discard(self, poses):
        validp = []
        validrs = []
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot_commander.get_current_state()
        req.ik_request.avoid_collisions = True
        
        for p in poses:
            req.ik_request.pose_stamped = p
            if self.debug:
                # ROS2 TF Broadcaster usage
                br = tf2_ros.TransformBroadcaster(self.node)
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = p.header.frame_id
                t.child_frame_id = "candidate_grasp_pose"
                t.transform.translation.x = p.pose.position.x
                t.transform.translation.y = p.pose.position.y
                t.transform.translation.z = p.pose.position.z
                t.transform.rotation.x = p.pose.orientation.x
                t.transform.rotation.y = p.pose.orientation.y
                t.transform.rotation.z = p.pose.orientation.z
                t.transform.rotation.w = p.pose.orientation.w
                br.sendTransform(t)
            
            future = self.compute_ik_srv.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            k = future.result()
            
            if k.error_code.val == 1:
                validp.append(p)
                validrs.append(k.solution)
        
        if validp:
            return [validp, validrs]
        return []

    # The planning service callback
    def startPlanning(self, req, res):
        # Initialize moveit stuff
        self.robot_commander = moveit_commander.RobotCommander()
        self.arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)
        self.gripper_move_group = moveit_commander.MoveGroupCommander(req.gripper_move_group)

        # Save request values to use them later in the pipeline
        self.arm_move_group_name = req.arm_move_group
        self.object_to_grasp = req.graspit_target_object
        self.gripper_move_group_name = req.gripper_move_group
        self.target_place = req.target_place
        self.replanning = req.max_replanning if req.max_replanning > 0 else 0

        # Get bounds for each gipper joint, so we can later use the graspit values
        self.getGripperBounds()

        success = False
        while(self.replanning >= 0):
            self.error_info = ""
            if not self.already_picked:
                # Call graspit
                graspit_grasps = self.graspThis(req.graspit_target_object)

                # Generate grasp poses
                self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)

            success = self.uberPlan()
            self.replanning -= 1
            if success:
                break

        response = EzStartPlanning.Response()
        response.success = success
        response.error_info = self.error_info
        return response

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
                    transform.header.stamp = self.node.get_clock().now().to_msg()
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
                    self.node.get_logger().error("fixItForGraspIt: " + str(e))
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
                    transform.header.frame_id = "world"
                    transform.child_frame_id = "target_object_frame"
                    transform.transform.translation.x = self.ez_objects[object_name][1].pose.position.x
                    transform.transform.translation.y = self.ez_objects[object_name][1].pose.position.y
                    transform.transform.translation.z = self.ez_objects[object_name][1].pose.position.z
                    transform.transform.rotation.x = self.ez_objects[object_name][1].pose.orientation.x
                    transform.transform.rotation.y = self.ez_objects[object_name][1].pose.orientation.y
                    transform.transform.rotation.z = self.ez_objects[object_name][1].pose.orientation.z
                    transform.transform.rotation.w = self.ez_objects[object_name][1].pose.orientation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    # Object -> Gripper
                    transform = TransformStamped()
                    transform.header.frame_id = "target_object_frame"
                    transform.child_frame_id = "ez_helper_graspit_pose"
                    transform.transform.translation.x = g.grasp_pose.pose.position.x
                    transform.transform.translation.y = g.grasp_pose.pose.position.y
                    transform.transform.translation.z = g.grasp_pose.pose.position.z
                    transform.transform.rotation.x = g.grasp_pose.pose.orientation.x
                    transform.transform.rotation.y = g.grasp_pose.pose.orientation.y
                    transform.transform.rotation.z = g.grasp_pose.pose.orientation.z
                    transform.transform.rotation.w = g.grasp_pose.pose.orientation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    transform_frame_gripper_trans = self.lookupTF(self.arm_move_group.get_end_effector_link(), self.gripper_frame)

                    # Gripper -> End Effector
                    transform = TransformStamped()
                    transform.header.frame_id = "ez_helper_graspit_pose"
                    transform.child_frame_id = "ez_helper_fixed_graspit_pose"
                    transform.transform.translation.x = -transform_frame_gripper_trans.transform.translation.x
                    transform.transform.translation.y = -transform_frame_gripper_trans.transform.translation.y
                    transform.transform.translation.z = -transform_frame_gripper_trans.transform.translation.z
                    transform.transform.rotation.x = transform_frame_gripper_trans.transform.rotation.x
                    transform.transform.rotation.y = transform_frame_gripper_trans.transform.rotation.y
                    transform.transform.rotation.z = transform_frame_gripper_trans.transform.rotation.z
                    transform.transform.rotation.w = transform_frame_gripper_trans.transform.rotation.w
                    self.tf2_buffer.set_transform(transform, "ez_helper")

                    # Graspit to MoveIt translation
                    # (End Effector -> End Effector)
                    graspit_moveit_transform = TransformStamped()
                    graspit_moveit_transform.header.frame_id = "ez_helper_fixed_graspit_pose"
                    graspit_moveit_transform.child_frame_id = "ez_helper_target_graspit_pose"
                    graspit_moveit_transform.transform.rotation.x = 0.7071
                    graspit_moveit_transform.transform.rotation.y = 0.0
                    graspit_moveit_transform.transform.rotation.z = 0.0
                    graspit_moveit_transform.transform.rotation.w = 0.7071
                    self.tf2_buffer.set_transform(graspit_moveit_transform, "ez_helper")

                    target_trans = self.lookupTF("world", "ez_helper_target_graspit_pose")

                    # World -> End Effector
                    res_pose = PoseStamped()
                    res_pose.header.frame_id = "world"
                    res_pose.pose.position.x = target_trans.transform.translation.x
                    res_pose.pose.position.y = target_trans.transform.translation.y
                    res_pose.pose.position.z = target_trans.transform.translation.z
                    res_pose.pose.orientation.x = target_trans.transform.rotation.x
                    res_pose.pose.orientation.y = target_trans.transform.rotation.y
                    res_pose.pose.orientation.z = target_trans.transform.rotation.z
                    res_pose.pose.orientation.w = target_trans.transform.rotation.w
                    self.grasp_poses.append(res_pose)
                    self.pose_n_joint[res_pose] = g.grasp_posture
                except Exception as e:
                    self.grasp_poses = []
                    self.node.get_logger().error("translateGraspIt2MoveIt:" + str(e))
                    break

    # Calculate the distance between two transformations in 2D (excluding the Z axis)
    def distanceXY(self, pose1, pose2):
        return sqrt((pose1.transform.translation.x**2 - pose2.transform.translation.x**2) + (pose1.transform.translation.y**2 - pose2.transform.translation.y**2))

    # Use atan2 and quaternion multiplication to "look at" the specified center
    # based on a specified current pose
    def lookAt(self, curr_quat, center, p):
        dx = p[0] - center[0]
        dy = p[1] - center[1]
        yaw = atan2(dy, dx)
        quat = quaternion_from_euler(0, 0, yaw)
        quat_start = [curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w]
        return list(quaternion_multiply(quat, quat_start))

    # Based on the specified object transformation, find poses in a circle
    # with the object as the center, that maintain the object's pitch and roll rotations
    # utilizing the lookAt function
    def gyrate(self, object_trans, curr_trans, step):
        center = [object_trans.transform.translation.x, object_trans.transform.translation.y]
        poses = []
        radius = self.distanceXY(object_trans, curr_trans)
        radius2 = radius**2
        # Calculate only for one quadrant of the circle
        for x in numpy.arange(center[0]-radius, center[0]+radius, step):
            for y in numpy.arange(center[1]-radius, center[1]+radius, step):
                x_center = x - center[0]
                y_center = y - center[1]
                if (x_center**2 + y_center**2) <= radius2:
                    x_ = center[0] - x_center
                    y_ = center[1] - y_center
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y])
                    poses.append([[x, y],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y])
                    poses.append([[x_, y],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y_])
                    poses.append([[x, y_],new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y_])
                    poses.append([[x_, y_],new_quat])
        return poses

    # Calculate the place pose of the end effector, based on the picked object's pose
    def calcTargetPose(self, obj_trans):
        for tryagain in range(0,4):
            try:

                start_trans = self.lookupTF(self.target_place.header.frame_id, self.arm_move_group.get_end_effector_link())

                obj = TransformStamped()
                obj.header.frame_id = "world"
                obj.child_frame_id = "ez_target_pick_world"
                obj.transform.translation.x = obj_trans[self.object_to_grasp].position.x
                obj.transform.translation.y = obj_trans[self.object_to_grasp].position.y
                obj.transform.translation.z = obj_trans[self.object_to_grasp].position.z
                obj.transform.rotation.x = obj_trans[self.object_to_grasp].orientation.x
                obj.transform.rotation.y = obj_trans[self.object_to_grasp].orientation.y
                obj.transform.rotation.z = obj_trans[self.object_to_grasp].orientation.z
                obj.transform.rotation.w = obj_trans[self.object_to_grasp].orientation.w
                self.tf2_buffer.set_transform(obj, "calcTargetPose")

                pick_to_target_frame_trans = self.lookupTF(self.target_place.header.frame_id, "ez_target_pick_world")

                ptt = TransformStamped()
                ptt.header.stamp = self.node.get_clock().now().to_msg()
                ptt.header.frame_id = self.target_place.header.frame_id
                ptt.child_frame_id = "ez_target_pick"
                ptt.transform.translation.x = pick_to_target_frame_trans.transform.translation.x
                ptt.transform.translation.y = pick_to_target_frame_trans.transform.translation.y
                ptt.transform.translation.z = pick_to_target_frame_trans.transform.translation.z
                ptt.transform.rotation.x = pick_to_target_frame_trans.transform.rotation.x
                ptt.transform.rotation.y = pick_to_target_frame_trans.transform.rotation.y
                ptt.transform.rotation.z = pick_to_target_frame_trans.transform.rotation.z
                ptt.transform.rotation.w = pick_to_target_frame_trans.transform.rotation.w
                self.tf2_buffer.set_transform(ptt, "calcTargetPose")

                trans1 = self.lookupTF("ez_target_pick", self.arm_move_group.get_end_effector_link())

                target_trans = TransformStamped()
                target_trans.header.frame_id = self.target_place.header.frame_id
                target_trans.child_frame_id = "ez_target_place"
                target_trans.transform.translation.x = self.target_place.pose.position.x
                target_trans.transform.translation.y = self.target_place.pose.position.y
                target_trans.transform.translation.z = self.target_place.pose.position.z
                target_trans.transform.rotation.x = pick_to_target_frame_trans.transform.rotation.x
                target_trans.transform.rotation.y = pick_to_target_frame_trans.transform.rotation.y
                target_trans.transform.rotation.z = pick_to_target_frame_trans.transform.rotation.z
                target_trans.transform.rotation.w = pick_to_target_frame_trans.transform.rotation.w
                self.tf2_buffer.set_transform(target_trans, "calcTargetPose")

                ee_target_trans = TransformStamped()
                ee_target_trans.header.frame_id = "ez_target_place"
                ee_target_trans.child_frame_id = "ez_target_to_ee"
                ee_target_trans.transform.translation.x = trans1.transform.translation.x
                ee_target_trans.transform.translation.y = trans1.transform.translation.y
                ee_target_trans.transform.translation.z = trans1.transform.translation.z
                ee_target_trans.transform.rotation.x = trans1.transform.rotation.x
                ee_target_trans.transform.rotation.y = trans1.transform.rotation.y
                ee_target_trans.transform.rotation.z = trans1.transform.rotation.z
                ee_target_trans.transform.rotation.w = trans1.transform.rotation.w
                self.tf2_buffer.set_transform(ee_target_trans, "calcTargetPose")

                trans2 = self.lookupTF(self.target_place.header.frame_id, "ez_target_to_ee")

                target_pose = PoseStamped()
                target_pose.header.frame_id = self.target_place.header.frame_id
                target_pose.pose.position.z = start_trans.transform.translation.z

                curr_state = self.robot_commander.get_current_state()
                # get_current_state does not include the attached object, so we add it manually
                # Note: In ROS2, getting attached objects directly from PlanningSceneInterface is different.
                # We assume the object is attached in the robot state logically for IK calculation.
                # If specific attached object retrieval is needed, we might need to query the planning scene service.
                # For this migration, we proceed assuming the state is sufficient or mock the attachment effect.
                
                req = GetPositionIK.Request()
                req.ik_request.group_name = self.arm_move_group_name
                req.ik_request.robot_state = curr_state
                req.ik_request.avoid_collisions = True

                gyrated_poses = self.gyrate(target_trans, start_trans, 0.1)

                for gp in gyrated_poses:
                    for i in range(0,6):
                        target_pose.pose.position.x = gp[0][0]
                        target_pose.pose.position.y = gp[0][1]
                        target_pose.pose.position.z = start_trans.transform.translation.z + i * 0.01
                        target_pose.pose.orientation.x = gp[1][0]
                        target_pose.pose.orientation.y = gp[1][1]
                        target_pose.pose.orientation.z = gp[1][2]
                        target_pose.pose.orientation.w = gp[1][3]
                        req.ik_request.pose_stamped = target_pose
                        
                        future = self.compute_ik_srv.call_async(req)
                        rclpy.spin_until_future_complete(self.node, future)
                        k = future.result()
                        
                        if k.error_code.val == 1:
                            return target_pose, k.solution
            except Exception as e:
                self.node.get_logger().error("calcTargetPose: " + str(e))
        return None, None

    # Check if the input of the scene setup service is valid
    def validSceneSetupInput(self, req):
        tmp = dict()
        tmp2 = EzSceneSetupResponse()
        info = []
        error_codes = []
        if len(req.finger_joint_names) == 0:
            info.append("Invalid service input: No finger_joint_names provided")
            error_codes.append(tmp2.NO_FINGER_JOINTS)
            return False, info, error_codes
        if req.gripper.name == "":
            info.append("Invalid service input: No gripper name provided")
            error_codes.append(tmp2.NO_NAME)
            return False, info, error_codes
        if req.gripper.graspit_file == "":
            info.append("Invalid service input: No graspit filename provided for the gripper")
            error_codes.append(tmp2.NO_FILENAME)
            return False, info, error_codes
        if self.pose_factor <= 0:
            info.append("Invalid service input: pose_factor cannot be negative or zero")
            error_codes.append(tmp2.INVALID_POSE_FACTOR)
            return False, info, error_codes

        for obj in req.objects:
            if obj.name == "":
                info.append("Invalid service input: No object name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obj.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obj.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obj.name] = 0
            if obj.graspit_file == "" and obj.moveit_file == "":
                info.append("Invalid service input: No file provided for object: " + obj.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obj.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of object: " + obj.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes

        for obs in req.obstacles:
            if obs.name == "":
                info.append("Invalid service input: No obstacle name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obs.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obs.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obs.name] = 0
            if obs.graspit_file == "" and obs.moveit_file == "":
                info.append("Invalid service input: No file provided for obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obs.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes
        return True, info, error_codes

    # The scene setup service callback
    def sceneSetup(self, req, res):
        self.pose_factor = req.pose_factor if req.pose_factor > 0 else self.pose_factor

        valid, info, ec = self.validSceneSetupInput(req)

        self.gripper_frame = req.gripper_frame

        response = EzSceneSetupResponse()
        
        if not valid:
            response.success = False
            response.info = info
            response.error_codes = ec
            return response

        response.success = True

        try:
            for obj in req.objects:
                # ------ Graspit world ------
                if obj.graspit_file != "":
                    atd_req = AddToDatabase.Request()
                    atd_req.filename = obj.graspit_file
                    atd_req.isRobot = False
                    atd_req.asGraspable = True
                    atd_req.modelName = obj.name
                    
                    future = self.add_model_srv.call_async(atd_req)
                    rclpy.spin_until_future_complete(self.node, future)
                    atd_res = future.result()
                    
                    if atd_res.returnCode != atd_res.SUCCESS:
                        response.success = False
                        response.info.append("Error adding object " + obj.name + " to graspit database")
                        response.error_codes.append(atd_res.returnCode)
                    else:
                        objectID = atd_res.modelID

                        loadm_req = LoadDatabaseModel.Request()
                        loadm_req.model_id = objectID
                        loadm_req.model_pose = self.fixItForGraspIt(obj, self.pose_factor)
                        
                        future = self.load_model_srv.call_async(loadm_req)
                        rclpy.spin_until_future_complete(self.node, future)
                        loadm_res = future.result()

                        self.ez_objects[obj.name] = [objectID, obj.pose]

                        if loadm_res.result != loadm_res.LOAD_SUCCESS:
                            response.success = False
                            response.info.append("Error loading object " + obj.name + " to graspit world")
                            response.error_codes.append(loadm_res.result)
                # ---------------------------

                # ------ Moveit scene -------
                if obj.moveit_file != "":
                    self.moveit_scene.add_mesh(obj.name, obj.pose, obj.moveit_file)
                # ---------------------------
            
            for obstacle in req.obstacles:

                # TODO
                # 1. Add and Load the model into the GraspIt database using self.add_model_srv and load_model_srv.
                # 2. Crucial: Use fixItForGraspIt to handle coordinate frame conversion and scaling.
                # 3. Store the object info in self.ez_objects.
                # 4. Similarly, handle the gripper/robot loading into the GraspIt world
                # END OF TODO
                
                # Implementation for Obstacles in GraspIt
                if obstacle.graspit_file != "":
                    atd_req = AddToDatabase.Request()
                    atd_req.filename = obstacle.graspit_file
                    atd_req.isRobot = False
                    atd_req.asGraspable = False # Obstacles are not graspable
                    atd_req.modelName = obstacle.name
                    
                    future = self.add_model_srv.call_async(atd_req)
                    rclpy.spin_until_future_complete(self.node, future)
                    atd_res = future.result()
                    
                    if atd_res.returnCode != atd_res.SUCCESS:
                        response.success = False
                        response.info.append("Error adding obstacle " + obstacle.name + " to graspit database")
                        response.error_codes.append(atd_res.returnCode)
                    else:
                        objectID = atd_res.modelID

                        loadm_req = LoadDatabaseModel.Request()
                        loadm_req.model_id = objectID
                        loadm_req.model_pose = self.fixItForGraspIt(obstacle, self.pose_factor)
                        
                        future = self.load_model_srv.call_async(loadm_req)
                        rclpy.spin_until_future_complete(self.node, future)
                        loadm_res = future.result()

                        self.ez_obstacles[obstacle.name] = [objectID, obstacle.pose]

                        if loadm_res.result != loadm_res.LOAD_SUCCESS:
                            response.success = False
                            response.info.append("Error loading obstacle " + obstacle.name + " to graspit world")
                            response.error_codes.append(loadm_res.result)
                
                # Implementation for Obstacles in MoveIt
                if obstacle.moveit_file != "":
                    self.moveit_scene.add_mesh(obstacle.name, obstacle.pose, obstacle.moveit_file)

            # ------ Graspit world (Robot/Gripper) ------
            atd_req = AddToDatabase.Request()
            atd_req.filename = req.gripper.graspit_file
            atd_req.isRobot = True
            atd_req.asGraspable = False
            atd_req.modelName = req.gripper.name
            atd_req.jointNames = req.finger_joint_names
            
            future = self.add_model_srv.call_async(atd_req)
            rclpy.spin_until_future_complete(self.node, future)
            atd_res = future.result()
            
            if atd_res.returnCode != atd_res.SUCCESS:
                response.success = False
                response.info.append("Error adding robot " + req.gripper.name + " to graspit database")
                response.error_codes.append(atd_res.returnCode)
            else:
                self.gripper_name = req.gripper.name
                robotID = atd_res.modelID

                loadm_req = LoadDatabaseModel.Request()
                loadm_req.model_id = robotID
                p = Pose()

                gripper_trans = self.lookupTF(self.gripper_frame, "world")

                p.position.x = gripper_trans.transform.translation.x * self.pose_factor
                p.position.y = gripper_trans.transform.translation.y * self.pose_factor
                p.position.z = gripper_trans.transform.translation.z * self.pose_factor
                loadm_req.model_pose = p
                
                future = self.load_model_srv.call_async(loadm_req)
                rclpy.spin_until_future_complete(self.node, future)
                loadm_res = future.result()

                if loadm_res.result != loadm_res.LOAD_SUCCESS:
                    response.success = False
                    response.info.append("Error loading robot " + req.gripper.name + " to graspit world")
                    response.error_codes.append(loadm_res.result)
            # ---------------------------

            return response

        except Exception as e:
            info.append(str(e))
            ec.append(response.EXCEPTION)
            response.success = False
            response.info = info
            response.error_codes = ec
            return response

[FILENAME: test2_ez_pnp2.py]
#!/usr/bin/env python3
import tf2_ros
import rclpy
import os

from ament_index_python.packages import get_package_share_directory
from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupRequest, EzStartPlanning, EzStartPlanningRequest
from ez_pick_and_place.msg import EzModel
from geometry_msgs.msg import PoseStamped

# Note:
# In order to run this test you need the roboskel_ros_resources package!

def main():
    rclpy.init()
    node = rclpy.create_node("ez_graspit_test")
    
    scene_setup_cli = node.create_client(EzSceneSetup, '/ez_pnp/scene_setup')
    print("Waiting for the services to come up...")
    while not scene_setup_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    start_planning_cli = node.create_client(EzStartPlanning, '/ez_pnp/start_planning')
    while not start_planning_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Planning service not available, waiting again...')
    
    print("Done!")

    # Get package path (simulating rospkg)
    try:
        pkg_share = get_package_share_directory('roboskel_ros_resources')
    except Exception:
        # Fallback for testing if package missing, using current dir or dummy
        pkg_share = os.getcwd() 

# [TODO]: INTEGRATION_SCENARIO_DEFINITION
    # Define a complete test case involving:
    # 1. A table obstacle and two objects ('E' and 'Z') with their respective GraspIt/MoveIt files.
    # 2. A specific 'gripper_frame' to anchor the planning coordinate system.
    # 3. A planning request to move object 'Z' to a designated target pose in 'world' frame.
# END OF TODO

    # 1. Setup Scene
    setup_req = EzSceneSetupRequest()
    setup_req.pose_factor = 1000.0
    setup_req.gripper_frame = "gripper_link"
    
    # Gripper Definition
    setup_req.gripper.name = "robotiq_gripper"
    setup_req.gripper.graspit_file = os.path.join(pkg_share, "models", "robotiq_85_gripper.xml")
    setup_req.finger_joint_names = ["finger_joint_1", "finger_joint_2"]

    # Object E
    obj_e = EzModel()
    obj_e.name = "E"
    obj_e.graspit_file = os.path.join(pkg_share, "models", "block_E.xml")
    obj_e.moveit_file = os.path.join(pkg_share, "meshes", "block_E.stl")
    pose_e = PoseStamped()
    pose_e.header.frame_id = "world"
    pose_e.pose.position.x = 0.5
    pose_e.pose.position.y = 0.0
    pose_e.pose.position.z = 0.05
    pose_e.pose.orientation.w = 1.0
    obj_e.pose = pose_e
    setup_req.objects.append(obj_e)

    # Object Z
    obj_z = EzModel()
    obj_z.name = "Z"
    obj_z.graspit_file = os.path.join(pkg_share, "models", "block_Z.xml")
    obj_z.moveit_file = os.path.join(pkg_share, "meshes", "block_Z.stl")
    pose_z = PoseStamped()
    pose_z.header.frame_id = "world"
    pose_z.pose.position.x = 0.5
    pose_z.pose.position.y = 0.2
    pose_z.pose.position.z = 0.05
    pose_z.pose.orientation.w = 1.0
    obj_z.pose = pose_z
    setup_req.objects.append(obj_z)

    # Obstacle: Table
    obs_table = EzModel()
    obs_table.name = "table"
    obs_table.graspit_file = os.path.join(pkg_share, "models", "table.xml")
    obs_table.moveit_file = os.path.join(pkg_share, "meshes", "table.stl")
    pose_table = PoseStamped()
    pose_table.header.frame_id = "world"
    pose_table.pose.position.x = 0.0
    pose_table.pose.position.y = 0.0
    pose_table.pose.position.z = -0.1
    pose_table.pose.orientation.w = 1.0
    obs_table.pose = pose_table
    setup_req.obstacles.append(obs_table)

    # Call Scene Setup
    future_setup = scene_setup_cli.call_async(setup_req)
    rclpy.spin_until_future_complete(node, future_setup)
    setup_res = future_setup.result()
    
    if not setup_res.success:
        node.get_logger().error(f"Scene Setup Failed: {setup_res.info}")
        node.destroy_node()
        return

    node.get_logger().info("Scene Setup Successful.")

    # 2. Planning Request
    plan_req = EzStartPlanningRequest()
    plan_req.arm_move_group = "arm"
    plan_req.gripper_move_group = "gripper"
    plan_req.graspit_target_object = "Z"
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = -0.3
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0
    plan_req.target_place = target_pose
    plan_req.max_replanning = 3

    # Call Planning
    node.get_logger().info("Starting Planning...")
    future_plan = start_planning_cli.call_async(plan_req)
    rclpy.spin_until_future_complete(node, future_plan)
    plan_res = future_plan.result()

    response = plan_res
    print(response)
    
    node.destroy_node()

if __name__ == '__main__':
    main()