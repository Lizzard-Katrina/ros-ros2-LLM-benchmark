# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
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
PKG = 'pr2_doors_gazebo_demo'
NAME = 'test_door_no_executive'

import roslib
roslib.load_manifest(PKG)

import unittest, sys, os, math
import time
import rospy, rostest
from nav_msgs.msg import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from door_msgs.msg import Door, DoorAction, DoorGoal
from actionlib import *

TEST_DURATION = 60000.0


class TestDoorNoExecutive(unittest.TestCase):
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
        self.door = DoorGoal()
        self.door.door = prior_door

        self.move = MoveBaseGoal()
        self.move.target_pose.header.frame_id = 'odom_combined'
        self.move.target_pose.pose.position.x = 10
        self.move.target_pose.pose.position.y = 10
        self.move.target_pose.pose.orientation.w = 1
        
        # TODO:
        # Implement ROS2 ActionClient logic for Door and MoveBase actions
        # END OF TODO
        rospy.Subscriber("/test_output", String, self.stringOutput)


    def stringOutput(self, str):
        print str.data
        
    
    def test_door_no_executive(self):
       self.assert_(self.ac_door.send_goal_and_wait(self.door, rospy.Duration(TEST_DURATION), rospy.Duration(5.0)))
       self.assert_(self.ac_move.send_goal_and_wait(self.move, rospy.Duration(TEST_DURATION), rospy.Duration(5.0)))
    


if __name__ == '__main__':
    rospy.init_node('open_door_test', anonymous=True)
    rostest.run(PKG, sys.argv[0], TestDoorNoExecutive, sys.argv) #, text_mode=True)

----------------------------


# LLM Response

```python
#!/usr/bin/env python
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
PKG = 'pr2_doors_gazebo_demo'
NAME = 'test_door_no_executive'

import unittest, sys, os, math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.action import MoveBase
from door_msgs.action import Door
from door_msgs.msg import Door as DoorMsg # Renamed to avoid conflict with action

TEST_DURATION = 60.0 # ROS2 Duration is in seconds


class TestDoorNoExecutive(unittest.TestCase):
    def __init__(self, *args):
        super(TestDoorNoExecutive, self).__init__(*args)
        rclpy.init()
        self.node = rclpy.create_node('open_door_test')

        # initial door
        prior_door = DoorMsg()
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
        prior_door.rot_dir = DoorMsg.ROT_DIR_COUNTERCLOCKWISE
        prior_door.hinge = DoorMsg.HINGE_P2
        prior_door.header.frame_id = "base_footprint"
        self.door_goal = Door.Goal()
        self.door_goal.door = prior_door

        self.move_goal = MoveBase.Goal()
        self.move_goal.target_pose.header.frame_id = 'odom_combined'
        self.move_goal.target_pose.pose.position.x = 10.0
        self.move_goal.target_pose.pose.position.y = 10.0
        self.move_goal.target_pose.pose.orientation.w = 1.0
        
        # TODO:
        # Implement ROS2 ActionClient logic for Door and MoveBase actions
        self.ac_door = ActionClient(self.node, Door, 'door_action')
        self.ac_move = ActionClient(self.node, MoveBase, 'move_base')
        # END OF TODO
        self.node.create_subscription(String, "/test_output", self.stringOutput, 10)


    def stringOutput(self, str_msg):
        self.node.get_logger().info(str_msg.data)
        
    
    def test_door_no_executive(self):
        # Wait for action servers to be available
        self.assertTrue(self.ac_door.wait_for_server(timeout_sec=10.0), "Door action server not available")
        self.assertTrue(self.ac_move.wait_for_server(timeout_sec=10.0), "MoveBase action server not available")

        # Send Door goal
        self.node.get_logger().info("Sending Door goal...")
        future_door = self.ac_door.send_goal_async(self.door_goal)
        rclpy.spin_until_future_complete(self.node, future_door)
        goal_handle_door = future_door.result()
        self.assertIsNotNone(goal_handle_door, "Door goal was rejected by the server")
        self.assertTrue(goal_handle_door.accepted, "Door goal was not accepted")

        self.node.get_logger().info("Waiting for Door result...")
        future_door_result = goal_handle_door.get_result_async()
        rclpy.spin_until_future_complete(self.node, future_door_result)
        result_door = future_door_result.result().result
        self.assertEqual(result_door.door_state, DoorMsg.DOOR_OPEN, "Door did not open successfully")
        self.node.get_logger().info("Door action completed.")

        # Send MoveBase goal
        self.node.get_logger().info("Sending MoveBase goal...")
        future_move = self.ac_move.send_goal_async(self.move_goal)
        rclpy.spin_until_future_complete(self.node, future_move)
        goal_handle_move = future_move.result()
        self.assertIsNotNone(goal_handle_move, "MoveBase goal was rejected by the server")
        self.assertTrue(goal_handle_move.accepted, "MoveBase goal was not accepted")

        self.node.get_logger().info("Waiting for MoveBase result...")
        future_move_result = goal_handle_move.get_result_async()
        rclpy.spin_until_future_complete(self.node, future_move_result)
        result_move = future_move_result.result().result
        self.assertEqual(result_move.status, 3, "MoveBase did not succeed (status 3 means SUCCEEDED)") # Actionlib.GoalStatus.SUCCEEDED
        self.node.get_logger().info("MoveBase action completed.")


if __name__ == '__main__':
    # ROS2 does not have a direct equivalent of rostest.run for Python unit tests.
    # You typically run unit tests using `pytest` or `colcon test`.
    # For a simple unittest, you can run it directly.
    unittest.main()
```