# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import copy

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 35.0
	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 1  # 10
	pug.allowed_touch_objects = []
	pug.attached_object_touch_links = ['<octomap>']
	pug.attached_object_touch_links.extend(links_to_allow_contact)

	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 15.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 1
	placeg.allowed_touch_objects = ['<octomap>']
	placeg.allowed_touch_objects.extend(links_to_allow_contact)

	return placeg

class PickAndPlaceServer(object):
	def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps()
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
		self.pickup_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)
		self.place_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		self.scene = PlanningSceneInterface()
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)
		self.scene_srv.wait_for_service()
		rospy.loginfo("Connected.")

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")

		# Get the object size
		self.object_height = rospy.get_param('~object_height')
		self.object_width = rospy.get_param('~object_width')
		self.object_depth = rospy.get_param('~object_depth')

		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)
		self.pick_as.start()

		self.place_as = SimpleActionServer(
			'/place_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)
		self.place_as.start()

	def pick_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)

	def place_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.place_as.set_aborted(p_res)
		else:
			self.place_as.set_succeeded(p_res)

	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo(
			"Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

                '''TODO:
Implement a polling mechanism using a standard loop to query the planning scene.
Explicitly inspect the world collision objects and compare their identifiers 
against the target name until the object is detected.
                END OF TODO'''
		rospy.loginfo("'" + object_name + "'' is in scene!")

	def grasp_object(self, object_pose):
		rospy.loginfo("Removing any previous 'part' object")
                '''TODO:
Implement the pick pipeline. The logic must explicitly handle:
1. Scene cleanup and sequential population of the environment with the 
   target and its supporting surface.
2. Explicit synchronization with the planning scene before action execution.
3. Standard action goal handling and returning the execution status code.
                END OF TODO'''

		return result.error_code.val

	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")
'''TODO:
Implement the place pipeline with a conditional fallback structure. 
Explicitly attempt the operation with a primary planning group, and 
incorporate a retry logic using an alternative group if the first 
attempt does not return a success state.
                END OF TODO'''
		return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()

----------------------------


# LLM Response

#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import time
import copy
from copy import deepcopy
from random import shuffle

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from spherical_grasps_server import SphericalGrasps
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.action import PickUpPose
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
                     grasp_pose=PoseStamped(),
                     possible_grasps=[],
                     links_to_allow_contact=None):
    """ Create a PickupGoal with the provided data"""
    if links_to_allow_contact is None:
        links_to_allow_contact = []

    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 35.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 1
    pug.allowed_touch_objects = []
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)

    return pug


def createPlaceGoal(place_pose,
                    place_locations,
                    group="arm_torso",
                    target="part",
                    links_to_allow_contact=None):
    """Create PlaceGoal with the provided data"""
    if links_to_allow_contact is None:
        links_to_allow_contact = []

    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.attached_object_name = target
    placeg.place_locations = place_locations
    placeg.allowed_planning_time = 15.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 1
    placeg.allowed_touch_objects = ['<octomap>']
    placeg.allowed_touch_objects.extend(links_to_allow_contact)

    return placeg


class PickAndPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_server')
        self.get_logger().info("Initalizing PickAndPlaceServer...")
        self._cb_group = ReentrantCallbackGroup()

        self.sg = SphericalGrasps()

        self.get_logger().info("Connecting to pickup AS")
        self.pickup_ac = ActionClient(self, PickupAction, '/pickup', callback_group=self._cb_group)
        self.pickup_ac.wait_for_server()
        self.get_logger().info("Succesfully connected.")

        self.get_logger().info("Connecting to place AS")
        self.place_ac = ActionClient(self, PlaceAction, '/place', callback_group=self._cb_group)
        self.place_ac.wait_for_server()
        self.get_logger().info("Succesfully connected.")

        self.scene = PlanningSceneInterface()

        self.get_logger().info("Connecting to /get_planning_scene service")
        self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self._cb_group)
        while not self.scene_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_planning_scene service...")
        self.get_logger().info("Connected.")

        self.get_logger().info("Connecting to clear octomap service...")
        self.clear_octomap_srv = self.create_client(Empty, '/clear_octomap', callback_group=self._cb_group)
        while not self.clear_octomap_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /clear_octomap service...")
        self.get_logger().info("Connected!")

        self.declare_parameter('object_height', 0.0)
        self.declare_parameter('object_width', 0.0)
        self.declare_parameter('object_depth', 0.0)
        self.declare_parameter('links_to_allow_contact', None)

        self.object_height = self.get_parameter('object_height').value
        self.object_width = self.get_parameter('object_width').value
        self.object_depth = self.get_parameter('object_depth').value

        self.links_to_allow_contact = self.get_parameter('links_to_allow_contact').value
        if self.links_to_allow_contact is None:
            self.get_logger().warn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
            self.links_to_allow_contact = []
        else:
            self.get_logger().info("Found links to allow contacts: " + str(self.links_to_allow_contact))

        self.pick_as = ActionServer(
            self,
            PickUpPose,
            '/pickup_pose',
            execute_callback=self.pick_cb,
            callback_group=self._cb_group)

        self.place_as = ActionServer(
            self,
            PickUpPose,
            '/place_pose',
            execute_callback=self.place_cb,
            callback_group=self._cb_group)

    def _call_service_and_wait(self, client, request):
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        return future.result()

    def _send_action_goal_and_wait(self, client, goal_msg):
        send_goal_future = client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
        result_wrap = result_future.result()
        if result_wrap is None:
            return None
        return result_wrap.result

    def pick_cb(self, goal_handle):
        """
        :type goal: PickUpPose.Goal
        """
        goal = goal_handle.request
        error_code = self.grasp_object(goal.object_pose)
        p_res = PickUpPose.Result()
        p_res.error_code = error_code
        if error_code != 1:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return p_res

    def place_cb(self, goal_handle):
        """
        :type goal: PickUpPose.Goal
        """
        goal = goal_handle.request
        error_code = self.place_object(goal.object_pose)
        p_res = PickUpPose.Result()
        p_res.error_code = error_code
        if error_code != 1:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return p_res

    def wait_for_planning_scene_object(self, object_name='part'):
        self.get_logger().info(
            "Waiting for object '" + object_name + "'' to appear in planning scene...")
        gps_req = GetPlanningScene.Request()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        while rclpy.ok():
            gps_res = self._call_service_and_wait(self.scene_srv, gps_req)
            if gps_res is not None:
                world_objects = gps_res.scene.world.collision_objects
                for collision_object in world_objects:
                    if collision_object.id == object_name:
                        self.get_logger().info("'" + object_name + "'' is in scene!")
                        return
            time.sleep(0.5)

    def grasp_object(self, object_pose):
        self.get_logger().info("Removing any previous 'part' object")
        try:
            self.scene.remove_world_object('part')
        except Exception:
            pass
        try:
            self.scene.remove_world_object('table')
        except Exception:
            pass

        time.sleep(1.0)

        self.get_logger().info("Adding target object to planning scene")
        self.scene.add_box(
            'part',
            object_pose,
            size=(self.object_depth, self.object_width, self.object_height)
        )

        table_pose = PoseStamped()
        table_pose.header = copy.deepcopy(object_pose.header)
        table_pose.pose.position.x = object_pose.pose.position.x
        table_pose.pose.position.y = object_pose.pose.position.y
        table_pose.pose.position.z = object_pose.pose.position.z - (self.object_height / 2.0) - 0.01
        table_pose.pose.orientation.w = 1.0

        self.get_logger().info("Adding supporting surface to planning scene")
        self.scene.add_box(
            'table',
            table_pose,
            size=(1.5, 1.5, 0.01)
        )

        self.wait_for_planning_scene_object('part')
        self.wait_for_planning_scene_object('table')

        self.get_logger().info("Generating grasps")
        possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
        shuffle(possible_grasps)

        pickup_goal = createPickupGoal(
            group="arm_torso",
            target="part",
            grasp_pose=object_pose,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )

        self.get_logger().info("Sending pickup goal")
        result = self._send_action_goal_and_wait(self.pickup_ac, pickup_goal)
        if result is None:
            return MoveItErrorCodes.FAILURE

        return result.error_code.val

    def place_object(self, object_pose):
        self.get_logger().info("Clearing octomap")
        self._call_service_and_wait(self.clear_octomap_srv, Empty.Request())

        place_l = PlaceLocation()
        place_l.place_pose = object_pose
        place_l.pre_place_approach = self.sg.createGripperTranslation(
            Vector3Stamped(
                header=object_pose.header,
                vector=Vector3(x=0.0, y=0.0, z=-1.0)
            ),
            0.10,
            0.15
        )
        place_l.post_place_retreat = self.sg.createGripperTranslation(
            Vector3Stamped(
                header=object_pose.header,
                vector=Vector3(x=0.0, y=0.0, z=1.0)
            ),
            0.10,
            0.15
        )
        place_l.post_place_posture = self.sg.create_open_gripper_posture()
        place_locations = [place_l]

        primary_goal = createPlaceGoal(
            object_pose,
            place_locations,
            group="arm",
            target="part",
            links_to_allow_contact=self.links_to_allow_contact
        )

        self.get_logger().info("Sending place goal with primary group")
        result = self._send_action_goal_and_wait(self.place_ac, primary_goal)

        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn("Primary place attempt failed, retrying with fallback group")
            fallback_goal = createPlaceGoal(
                object_pose,
                place_locations,
                group="arm_torso",
                target="part",
                links_to_allow_contact=self.links_to_allow_contact
            )
            result = self._send_action_goal_and_wait(self.place_ac, fallback_goal)
            if result is None:
                return MoveItErrorCodes.FAILURE

        return result.error_code.val


if __name__ == '__main__':
    rclpy.init()
    paps = PickAndPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(paps)
    try:
        executor.spin()
    finally:
        paps.destroy_node()
        rclpy.shutdown()