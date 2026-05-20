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

FILE_PATH: pick_and_place_server.py
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

[FILENAME: pick_and_place_server.py]
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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from spherical_grasps_server import SphericalGrasps
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, MoveItErrorCodes, PlaceLocation
from moveit_msgs.action import Pickup, Place
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.action import PickUpPose
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty
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
	pug = Pickup.Goal()
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
	if links_to_allow_contact:
		pug.attached_object_touch_links.extend(links_to_allow_contact)

	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = Place.Goal()
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
	if links_to_allow_contact:
		placeg.allowed_touch_objects.extend(links_to_allow_contact)

	return placeg

class PickAndPlaceServer(Node):
	def __init__(self):
		super().__init__('pick_and_place_server')
		self.get_logger().info("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps(self)
		self.get_logger().info("Connecting to pickup AS")
		self.pickup_ac = ActionClient(self, Pickup, '/pickup')
		self.pickup_ac.wait_for_server()
		self.get_logger().info("Succesfully connected.")
		self.get_logger().info("Connecting to place AS")
		self.place_ac = ActionClient(self, Place, '/place')
		self.place_ac.wait_for_server()
		self.get_logger().info("Succesfully connected.")
		self.scene = PlanningSceneInterface()
		self.get_logger().info("Connecting to /get_planning_scene service")
		self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene')
		self.scene_srv.wait_for_service()
		self.get_logger().info("Connected.")

		self.get_logger().info("Connecting to clear octomap service...")
		self.clear_octomap_srv = self.create_client(Empty, '/clear_octomap')
		self.clear_octomap_srv.wait_for_service()
		self.get_logger().info("Connected!")

		# Get the object size
		self.declare_parameter('object_height', 0.1)
		self.declare_parameter('object_width', 0.05)
		self.declare_parameter('object_depth', 0.05)
		self.declare_parameter('links_to_allow_contact', [''])
		
		self.object_height = self.get_parameter('object_height').value
		self.object_width = self.get_parameter('object_width').value
		self.object_depth = self.get_parameter('object_depth').value

		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = self.get_parameter('links_to_allow_contact').value
		if not self.links_to_allow_contact or self.links_to_allow_contact == ['']:
			self.get_logger().warn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
			self.links_to_allow_contact = None
		else:
			self.get_logger().info("Found links to allow contacts: " + str(self.links_to_allow_contact))

		self.pick_as = ActionServer(
			self, PickUpPose, '/pickup_pose',
			execute_callback=self.pick_cb)

		self.place_as = ActionServer(
			self, PickUpPose, '/place_pose',
			execute_callback=self.place_cb)

	def pick_cb(self, goal_handle):
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

		part_in_scene = False
		while not part_in_scene and rclpy.ok():
			future = self.scene_srv.call_async(gps_req)
			rclpy.spin_until_future_complete(self, future)
			gps_resp = future.result()
			for collision_obj in gps_resp.scene.world.collision_objects:
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			if not part_in_scene:
				time.sleep(0.5)

		self.get_logger().info("'" + object_name + "'' is in scene!")

	def grasp_object(self, object_pose):
		self.get_logger().info("Removing any previous 'part' object")
		self.scene.remove_world_object("part")
		self.scene.remove_world_object("table")
		
		req = Empty.Request()
		future = self.clear_octomap_srv.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		time.sleep(1.0)
		
		table_pose = copy.deepcopy(object_pose)
		table_pose.pose.position.z -= (self.object_height / 2.0) + 0.05
		self.scene.add_box("table", table_pose, size=(1.0, 1.0, 0.1))
		self.scene.add_box("part", object_pose, size=(self.object_depth, self.object_width, self.object_height))
		
		self.wait_for_planning_scene_object("part")
		
		possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
		goal = createPickupGoal("arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)
		
		future = self.pickup_ac.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, future)
		goal_handle = future.result()
		
		if not goal_handle.accepted:
			return 0
			
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		result = result_future.result().result

		return result.error_code.val

	def place_object(self, object_pose):
		self.get_logger().info("Clearing octomap")
		req = Empty.Request()
		future = self.clear_octomap_srv.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		
		place_locations = self.sg.create_placelocations_from_object_pose(object_pose)
		goal = createPlaceGoal(object_pose, place_locations, "arm_torso", "part", self.links_to_allow_contact)
		
		future = self.place_ac.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, future)
		goal_handle = future.result()
		
		if not goal_handle.accepted:
			return 0
			
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		result = result_future.result().result
		
		if result.error_code.val != 1:
			self.get_logger().warn("Failed with arm_torso, retrying with arm group")
			goal = createPlaceGoal(object_pose, place_locations, "arm", "part", self.links_to_allow_contact)
			future = self.place_ac.send_goal_async(goal)
			rclpy.spin_until_future_complete(self, future)
			goal_handle = future.result()
			if goal_handle.accepted:
				result_future = goal_handle.get_result_async()
				rclpy.spin_until_future_complete(self, result_future)
				result = result_future.result().result

		return result.error_code.val


if __name__ == '__main__':
	rclpy.init()
	paps = PickAndPlaceServer()
	rclpy.spin(paps)
	rclpy.shutdown()