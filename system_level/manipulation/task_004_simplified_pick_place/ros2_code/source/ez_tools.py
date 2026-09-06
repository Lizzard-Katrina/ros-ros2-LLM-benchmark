#!/usr/bin/env python3
import time
import numpy
import rclpy
import tf2_ros

from math import sqrt, atan2
from task_004_simplified_pick_place.quat_helpers import euler2quat, qmult

from geometry_msgs.msg import TransformStamped, PoseStamped, Pose


class EZToolSet():

    object_to_grasp = ""
    arm_move_group = None
    robot_commander = None
    gripper_move_group = ""
    arm_move_group_name = ""
    gripper_move_group_name = ""

    pose_factor = 1000

    node = None

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

    def initTF2(self):
        """Initialize tf2_ros.Buffer() and TransformListener on the node."""
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self.node)

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

    def grab(self, graspit_result):
        self.attachThis(self.object_to_grasp)
        res = self.moveGripper(graspit_result)
        self.detachThis(self.object_to_grasp)
        return res

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

    def moveGripperToState(self, state):
        self.gripper_move_group.set_joint_value_target(state)
        return self.gripper_move_group.go()

    # Shortcut of tf's lookup_transform
    def lookupTF(self, target_frame, source_frame):
        return self.tf2_buffer.lookup_transform(
            target_frame, source_frame,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=10))

    def graspThis(self, object_name):
        from moveit_msgs.msg import CollisionObject
        target = CollisionObject()
        target.id = str(self.ez_objects[object_name][0])
        target.primitive_poses = [self.ez_objects[object_name][1].pose]
        req = self._make_grasp_planning_request(
            group_name=self.gripper_name, target=target)
        future = self.planning_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        return response.grasps

    def _make_grasp_planning_request(self, group_name, target):
        from moveit_msgs.srv import GraspPlanning
        req = GraspPlanning.Request()
        req.group_name = group_name
        req.target = target
        return req

    def attachThis(self, object_name):
        touch_links = self.robot_commander.get_link_names(self.gripper_move_group_name)
        self.arm_move_group.attach_object(object_name, link_name=self.arm_move_group.get_end_effector_link(), touch_links=touch_links)

    def detachThis(self, object_name):
        self.arm_move_group.detach_object(object_name)

    def uberPlan(self):
        return self.pick() and self.place()

    def pick(self):
        if not self.already_picked:
            self.openGripper()
            time.sleep(1)
            valid_g = self.discard(self.grasp_poses)

            if len(valid_g) > 0:
                for j in range(len(valid_g[0])):
                    self.arm_move_group.set_start_state_to_current_state()
                    if self.move(valid_g[0][j].pose):
                        time.sleep(1)
                        result = self.grab(self.pose_n_joint[valid_g[0][j]])
                        self.attachThis(self.object_to_grasp)
                        return result
                self.error_info = "Error while trying to pick the object!"
            else:
                self.error_info = "No valid grasps were found!"
            return False
        return True

    def place(self):
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
            self.error_info = "Error while calculating the target pose!"
        return False

    def getGripperBounds(self):
        for joint in self.gripper_move_group.get_joints():
            self.gripper_joint_bounds[joint] = self.robot_commander.get_joint(joint).max_bound()

    def discard(self, poses):
        from moveit_msgs.srv import GetPositionIK
        validp = []
        validrs = []
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot_commander.get_current_state()
        req.ik_request.avoid_collisions = True
        for p in poses:
            req.ik_request.pose_stamped = p
            future = self.compute_ik_srv.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            k = future.result()
            if k.error_code.val == 1:
                validp.append(p)
                validrs.append(k.solution)
        if validp:
            return [validp, validrs]
        return []

    def startPlanning(self, request, response):
        req = request
        self.arm_move_group_name = req.arm_move_group
        self.object_to_grasp = req.graspit_target_object
        self.gripper_move_group_name = req.gripper_move_group
        self.target_place = req.target_place
        self.replanning = req.max_replanning if req.max_replanning > 0 else 0

        self.getGripperBounds()

        res = False
        while(self.replanning >= 0):
            self.error_info = ""
            if not self.already_picked:
                graspit_grasps = self.graspThis(req.graspit_target_object)
                self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)

            res = self.uberPlan()
            self.replanning -= 1
            if res:
                break

        response.success = res
        response.error_info = self.error_info
        return response

    def fixItForGraspIt(self, obj, pose_factor):
        for tryagain in range(0, 4):
            p = Pose()
            if obj.pose.header.frame_id == "world":
                p.position.x = obj.pose.pose.position.x * pose_factor
                p.position.y = obj.pose.pose.position.y * pose_factor
                p.position.z = obj.pose.pose.position.z * pose_factor
                p.orientation.x = obj.pose.pose.orientation.x
                p.orientation.y = obj.pose.pose.orientation.y
                p.orientation.z = obj.pose.pose.orientation.z
                p.orientation.w = obj.pose.pose.orientation.w
                return p
            else:
                try:
                    transform = TransformStamped()
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

    def translateGraspIt2MoveIt(self, grasps, object_name):
        for tryagain in range(0, 4):
            self.grasp_poses = []
            for g in grasps:
                try:
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

                    graspit_moveit_transform = TransformStamped()
                    graspit_moveit_transform.header.frame_id = "ez_helper_fixed_graspit_pose"
                    graspit_moveit_transform.child_frame_id = "ez_helper_target_graspit_pose"
                    graspit_moveit_transform.transform.rotation.x = 0.7071
                    graspit_moveit_transform.transform.rotation.y = 0.0
                    graspit_moveit_transform.transform.rotation.z = 0.0
                    graspit_moveit_transform.transform.rotation.w = 0.7071
                    self.tf2_buffer.set_transform(graspit_moveit_transform, "ez_helper")

                    target_trans = self.lookupTF("world", "ez_helper_target_graspit_pose")

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
                    print("translateGraspIt2MoveIt:" + str(e))
                    break

    def distanceXY(self, pose1, pose2):
        return sqrt((pose1.transform.translation.x**2 - pose2.transform.translation.x**2) + (pose1.transform.translation.y**2 - pose2.transform.translation.y**2))

    def lookAt(self, curr_quat, center, p):
        dx = p[0] - center[0]
        dy = p[1] - center[1]
        yaw = atan2(dy, dx)
        quat_wxyz = euler2quat(0, 0, yaw)
        quat = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
        quat_start = [curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w]
        result = qmult(
            [quat[3], quat[0], quat[1], quat[2]],
            [quat_start[3], quat_start[0], quat_start[1], quat_start[2]]
        )
        return [result[1], result[2], result[3], result[0]]

    def gyrate(self, object_trans, curr_trans, step):
        center = [object_trans.transform.translation.x, object_trans.transform.translation.y]
        poses = []
        radius = self.distanceXY(object_trans, curr_trans)
        radius2 = radius**2
        for x in numpy.arange(center[0]-radius, center[0]+radius, step):
            for y in numpy.arange(center[1]-radius, center[1]+radius, step):
                x_center = x - center[0]
                y_center = y - center[1]
                if (x_center**2 + y_center**2) <= radius2:
                    x_ = center[0] - x_center
                    y_ = center[1] - y_center
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y])
                    poses.append([[x, y], new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y])
                    poses.append([[x_, y], new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x, y_])
                    poses.append([[x, y_], new_quat])
                    new_quat = self.lookAt(curr_trans.transform.rotation, center, [x_, y_])
                    poses.append([[x_, y_], new_quat])
        return poses

    def calcTargetPose(self, obj_trans):
        from moveit_msgs.srv import GetPositionIK
        for tryagain in range(0, 4):
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
                attobj = self.moveit_scene.get_attached_objects([self.object_to_grasp])
                curr_state.attached_collision_objects = [attobj[self.object_to_grasp]]
                req = GetPositionIK.Request()
                req.ik_request.group_name = self.arm_move_group_name
                req.ik_request.robot_state = curr_state
                req.ik_request.avoid_collisions = True

                gyrated_poses = self.gyrate(target_trans, start_trans, 0.1)

                for gp in gyrated_poses:
                    for i in range(0, 6):
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
                print("calcTargetPose" + str(e))
        return None, None

    def validSceneSetupInput(self, req):
        tmp = dict()
        info = []
        error_codes = []
        if len(req.finger_joint_names) == 0:
            info.append("Invalid service input: No finger_joint_names provided")
            error_codes.append(-1)
            return False, info, error_codes
        if req.gripper.name == "":
            info.append("Invalid service input: No gripper name provided")
            error_codes.append(-2)
            return False, info, error_codes
        if req.gripper.graspit_file == "":
            info.append("Invalid service input: No graspit filename provided for the gripper")
            error_codes.append(-3)
            return False, info, error_codes
        if self.pose_factor <= 0:
            info.append("Invalid service input: pose_factor cannot be negative or zero")
            error_codes.append(-4)
            return False, info, error_codes

        for obj in req.objects:
            if obj.name == "":
                info.append("Invalid service input: No object name provided")
                error_codes.append(-2)
                return False, info, error_codes
            if obj.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obj.name)
                error_codes.append(-5)
                return False, info, error_codes
            else:
                tmp[obj.name] = 0
            if obj.graspit_file == "" and obj.moveit_file == "":
                info.append("Invalid service input: No file provided for object: " + obj.name)
                error_codes.append(-3)
                return False, info, error_codes
            if obj.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of object: " + obj.name)
                error_codes.append(-6)
                return False, info, error_codes

        for obs in req.obstacles:
            if obs.name == "":
                info.append("Invalid service input: No obstacle name provided")
                error_codes.append(-2)
                return False, info, error_codes
            if obs.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obs.name)
                error_codes.append(-5)
                return False, info, error_codes
            else:
                tmp[obs.name] = 0
            if obs.graspit_file == "" and obs.moveit_file == "":
                info.append("Invalid service input: No file provided for obstacle: " + obs.name)
                error_codes.append(-3)
                return False, info, error_codes
            if obs.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name)
                error_codes.append(-6)
                return False, info, error_codes
        return True, info, error_codes

    def sceneSetup(self, request, response):
        req = request
        self.pose_factor = req.pose_factor if req.pose_factor > 0 else self.pose_factor

        valid, info, ec = self.validSceneSetupInput(req)

        self.gripper_frame = req.gripper_frame

        if not valid:
            return response

        try:
            for obj in req.objects:
                if obj.graspit_file != "":
                    atd_req = self._make_add_to_database_request()
                    atd_req.filename = obj.graspit_file
                    atd_req.is_robot = False
                    atd_req.as_graspable = True
                    atd_req.model_name = obj.name
                    future = self.add_model_srv.call_async(atd_req)
                    rclpy.spin_until_future_complete(self.node, future)
                    add_response = future.result()

                    if add_response is not None:
                        objectID = add_response.model_id if hasattr(add_response, 'model_id') else 0

                        loadm_req = self._make_load_database_model_request()
                        loadm_req.model_id = objectID
                        loadm_req.model_pose = self.fixItForGraspIt(obj, self.pose_factor)
                        future = self.load_model_srv.call_async(loadm_req)
                        rclpy.spin_until_future_complete(self.node, future)

                        self.ez_objects[obj.name] = [objectID, obj.pose]

                if obj.moveit_file != "":
                    self.moveit_scene.add_mesh(obj.name, obj.pose, obj.moveit_file)

            for obstacle in req.obstacles:
                if obstacle.graspit_file != "":
                    atd_req = self._make_add_to_database_request()
                    atd_req.filename = obstacle.graspit_file
                    atd_req.is_robot = False
                    atd_req.as_graspable = False
                    atd_req.model_name = obstacle.name
                    future = self.add_model_srv.call_async(atd_req)
                    rclpy.spin_until_future_complete(self.node, future)
                    add_response = future.result()

                    if add_response is not None:
                        obstacleID = add_response.model_id if hasattr(add_response, 'model_id') else 0

                        loadm_req = self._make_load_database_model_request()
                        loadm_req.model_id = obstacleID
                        loadm_req.model_pose = self.fixItForGraspIt(obstacle, self.pose_factor)
                        future = self.load_model_srv.call_async(loadm_req)
                        rclpy.spin_until_future_complete(self.node, future)

                        self.ez_obstacles[obstacle.name] = [obstacleID, obstacle.pose]

                if obstacle.moveit_file != "":
                    self.moveit_scene.add_mesh(obstacle.name, obstacle.pose, obstacle.moveit_file)

            atd_req = self._make_add_to_database_request()
            atd_req.filename = req.gripper.graspit_file
            atd_req.is_robot = True
            atd_req.as_graspable = False
            atd_req.model_name = req.gripper.name
            atd_req.joint_names = req.finger_joint_names
            future = self.add_model_srv.call_async(atd_req)
            rclpy.spin_until_future_complete(self.node, future)
            add_response = future.result()

            if add_response is not None:
                self.gripper_name = req.gripper.name
                robotID = add_response.model_id if hasattr(add_response, 'model_id') else 0

                loadm_req = self._make_load_database_model_request()
                loadm_req.model_id = robotID
                p = Pose()

                gripper_trans = self.lookupTF(self.gripper_frame, "world")

                p.position.x = gripper_trans.transform.translation.x * self.pose_factor
                p.position.y = gripper_trans.transform.translation.y * self.pose_factor
                p.position.z = gripper_trans.transform.translation.z * self.pose_factor
                loadm_req.model_pose = p
                future = self.load_model_srv.call_async(loadm_req)
                rclpy.spin_until_future_complete(self.node, future)

            return response

        except Exception as e:
            print(str(e))
            return response

    def _make_add_to_database_request(self):
        class _Req:
            def __init__(self):
                self.filename = ""
                self.is_robot = False
                self.as_graspable = False
                self.model_name = ""
                self.joint_names = []
        return _Req()

    def _make_load_database_model_request(self):
        class _Req:
            def __init__(self):
                self.model_id = 0
                self.model_pose = Pose()
        return _Req()