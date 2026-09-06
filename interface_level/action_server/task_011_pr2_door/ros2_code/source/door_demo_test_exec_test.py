#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## Gazebo collision validation
PKG = 'task_011_pr2_door'
NAME = 'test_door_no_executive'

import unittest
import sys
import os
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from task_011_pr2_door.door_msgs import Door, DoorAction, DoorGoal
from task_011_pr2_door.move_base_msgs import MoveBaseAction, MoveBaseGoal
from task_011_pr2_door.action_client import ActionClient

TEST_DURATION = 60000.0


class TestDoorNoExecutive(unittest.TestCase):
    def __init__(self, *args):
        super(TestDoorNoExecutive, self).__init__(*args)

        # Ensure rclpy is initialized
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self.node = Node('open_door_test')

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
        self.door = DoorGoal()
        self.door.door = prior_door

        self.move = MoveBaseGoal()
        self.move.target_pose.header.frame_id = 'odom_combined'
        self.move.target_pose.pose.position.x = 10
        self.move.target_pose.pose.position.y = 10
        self.move.target_pose.pose.orientation.w = 1

        self.ac_door = ActionClient(self.node, 'move_through_door', DoorAction)
        print('Waiting for open door action server')
        self.ac_door.wait_for_server()

        self.ac_move = ActionClient(self.node, 'move_base_local', MoveBaseAction)
        print('Waiting for move base action server')
        self.ac_move.wait_for_server()

        self.node.create_subscription(
            String,
            '/test_output',
            self.stringOutput,
            10
        )
        # Keep reference: Subscriber('/test_output', String, ...)

    def stringOutput(self, msg):
        print(msg.data)

    def test_door_no_executive(self):
        self.assertTrue(self.ac_door.send_goal_and_wait(self.door, TEST_DURATION, 5.0))
        self.assertTrue(self.ac_move.send_goal_and_wait(self.move, TEST_DURATION, 5.0))


def main():
    rclpy.init(args=sys.argv)
    unittest.main(module=__name__, exit=False)
    rclpy.shutdown()


if __name__ == '__main__':
    main()