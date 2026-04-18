#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#

## Gazebo collision validation
PKG = 'pr2_doors_gazebo_demo'
NAME = 'test_door_no_executive'

import unittest
import sys
import os
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from move_base_msgs.action import MoveBase
from door_msgs.msg import Door
from door_msgs.action import Door as DoorAction

TEST_DURATION = 60000.0


class ActionClientCompat:
    def __init__(self, node, action_type, action_name):
        self._node = node
        self._client = ActionClient(node, action_type, action_name)

    def wait_for_server(self, timeout_sec=None):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal_and_wait(self, goal, execute_timeout, preempt_timeout):
        timeout_sec = None
        if isinstance(execute_timeout, Duration):
            timeout_sec = execute_timeout.nanoseconds / 1e9
        elif isinstance(execute_timeout, (int, float)):
            timeout_sec = float(execute_timeout)

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_goal_future, timeout_sec=timeout_sec)
        if not send_goal_future.done():
            return False

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            return False

        result = result_future.result()
        return result.status == GoalStatus.STATUS_SUCCEEDED


class TestDoorNoExecutive(unittest.TestCase):
    node = None

    def __init__(self, *args):
        super(TestDoorNoExecutive, self).__init__(*args)

        # initial door
        prior_door = Door()
        prior_door.frame_p1.x = 1.0
        prior_door.frame_p1.y = -0.5
        prior_door.frame_p2.x = 1.0
        prior_door.frame_p2.y = 0.5
        prior_door.door_p1.x = 1.0
        prior_door.door_p1.y = -0.5
        prior_door.door_p2.x = 1.0
        prior_door.door_p2.y = 0.5
        prior_door.travel_dir.x = 1.0
        prior_door.travel_dir.y = 0.0
        prior_door.travel_dir.z = 0.0
        prior_door.rot_dir = Door.ROT_DIR_COUNTERCLOCKWISE
        prior_door.hinge = Door.HINGE_P2
        prior_door.header.frame_id = "base_footprint"
        self.door = DoorAction.Goal()
        self.door.door = prior_door

        self.move = MoveBase.Goal()
        self.move.target_pose.header.frame_id = 'odom_combined'
        self.move.target_pose.pose.position.x = 10.0
        self.move.target_pose.pose.position.y = 10.0
        self.move.target_pose.pose.orientation.w = 1.0

        # TODO:
        # Implement ROS2 ActionClient logic for Door and MoveBase actions
        self.ac_door = ActionClientCompat(self.node, DoorAction, '/door_action')
        self.ac_move = ActionClientCompat(self.node, MoveBase, '/move_base')
        self.ac_door.wait_for_server(timeout_sec=30.0)
        self.ac_move.wait_for_server(timeout_sec=30.0)
        # END OF TODO
        self.node.create_subscription(String, "/test_output", self.stringOutput, 10)

    def stringOutput(self, str):
        print(str.data)

    def test_door_no_executive(self):
        self.assertTrue(
            self.ac_door.send_goal_and_wait(
                self.door,
                Duration(seconds=TEST_DURATION),
                Duration(seconds=5.0),
            )
        )
        self.assertTrue(
            self.ac_move.send_goal_and_wait(
                self.move,
                Duration(seconds=TEST_DURATION),
                Duration(seconds=5.0),
            )
        )


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    TestDoorNoExecutive.node = rclpy.create_node('open_door_test')
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestDoorNoExecutive)
    result = unittest.TextTestRunner().run(suite)
    TestDoorNoExecutive.node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result.wasSuccessful() else 1)