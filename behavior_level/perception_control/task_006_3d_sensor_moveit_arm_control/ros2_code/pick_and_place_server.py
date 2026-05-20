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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from spherical_grasps_server import SphericalGrasps
from moveit_commander import PlanningSceneInterface
from moveit_msgs.action import Pickup, Place
from moveit_msgs.msg import Grasp, MoveItErrorCodes, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.action import PickUpPose
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty
from copy import deepcopy
from random import shuffle
import copy
import time

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
            self, PickUpPose, '/pickup_pose', self.pick_cb)

        self.place_as = ActionServer(
            self, PickUpPose, '/place_pose', self.place_cb)

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
            response = future.result()
            if response is not None:
                known_objects = [obj.id for obj in response.scene.world.collision_objects]
                if object_name in known_objects:
                    part_in_scene = True
                else:
                    time.sleep(0.5)
            else:
                time.sleep(0.5)
        self.get_logger().info("'" + object_name + "'' is in scene!")

    def grasp_object(self, object_pose):
        self.get_logger().info("Removing any previous 'part' object")
        self.scene.remove_world_object("part")
        self.scene.remove_world_object("table")
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = object_pose.header.frame_id
        table_pose.pose.position.x = object_pose.pose.position.x
        table_pose.pose.position.y = object_pose.pose.position.y
        table_pose.pose.position.z = object_pose.pose.position.z - self.object_height/2.0 - 0.01
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", table_pose, size=(0.5, 0.5, 0.02))
        
        part_pose = PoseStamped()
        part_pose.header.frame_id = object_pose.header.frame_id
        part_pose.pose = object_pose.pose
        self.scene.add_box("part", part_pose, size=(self.object_depth, self.object_width, self.object_height))
        
        self.wait_for_planning_scene_object("part")
        
        possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
        goal = createPickupGoal("arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)
        
        future = self.pickup_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Pickup goal rejected")
            return 0
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result

        return result.error_code.val

    def place_object(self, object_pose):
        self.get_logger().info("Clearing octomap")
        req = Empty.Request()
        future_octo = self.clear_octomap_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future_octo)
        
        place_locations = self.sg.create_placelocations_from_object_pose(object_pose)
        goal = createPlaceGoal(object_pose, place_locations, "arm_torso", "part", self.links_to_allow_contact)
        
        future = self.place_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return 0
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val != 1:
            self.get_logger().warn("Place failed with arm_torso, retrying with arm group")
            goal = createPlaceGoal(object_pose, place_locations, "arm", "part", self.links_to_allow_contact)
            future = self.place_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                result = res_future.result().result

        return result.error_code.val


if __name__ == '__main__':
    rclpy.init()
    paps = PickAndPlaceServer()
    rclpy.spin(paps)
    paps.destroy_node()
    rclpy.shutdown()