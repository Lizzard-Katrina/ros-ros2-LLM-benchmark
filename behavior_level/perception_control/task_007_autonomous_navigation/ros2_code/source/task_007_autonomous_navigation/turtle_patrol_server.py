#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

import math
import threading
import time

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from turtlebot3_msgs.action import Patrol
    _HAS_PATROL = True
except ImportError:
    _HAS_PATROL = False


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print('TurtleBot3 Patrol Server')
        print('----------------------------------------------')

        if _HAS_PATROL:
            self._action_server = ActionServer(
                self,
                Patrol,
                'turtlebot3',
                self.execute_callback,
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self.goal_callback)

            self.goal_msg = Patrol.Goal()

        self.twist = Twist()
        self.odom = Odometry()
        self.position = Point()
        self.rotation = 0.0

        self.linear_x = 1.0
        self.angular_z = 4.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def go_front(self, position, length):
        """Spatial-feedback control loop that drives the robot forward for a
        specified distance using odometry-based Euclidean displacement."""
        initial_x = self.odom.pose.pose.position.x
        initial_y = self.odom.pose.pose.position.y

        timeout_seconds = 60.0
        start_time = time.time()

        while rclpy.ok():
            if (time.time() - start_time) > timeout_seconds:
                self.get_logger().warn('go_front timeout reached')
                break

            rclpy.spin_once(self, timeout_sec=0.1)

            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y
            distance = math.sqrt(
                (current_x - initial_x) ** 2 + (current_y - initial_y) ** 2
            )

            if distance >= length:
                break

            remaining = length - distance
            speed = min(self.linear_x, max(0.05, remaining * 0.5))

            self.twist.linear.x = speed
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(0.05)

        self.init_twist()

    def turn(self, target_angle):
        """Heading-feedback control loop to rotate the robot to a target
        relative angle using proportional control with atan2 normalization."""
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        Kp = 2.0
        timeout_seconds = 30.0
        start_time = time.time()

        while rclpy.ok():
            if (time.time() - start_time) > timeout_seconds:
                self.get_logger().warn('turn timeout reached')
                break

            rclpy.spin_once(self, timeout_sec=0.1)

            current_yaw = self.get_yaw()

            error = target_yaw - current_yaw
            angle_diff = math.atan2(math.sin(error), math.cos(error))

            if abs(angle_diff) < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = Kp * angle_diff
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(0.05)

        self.init_twist()

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Patrol.Feedback()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)

        while True:
            if self.goal_msg.goal.x == 1:
                for count in range(iteration):
                    self.square(feedback_msg, goal_handle, length)
                feedback_msg.state = 'square patrol complete!!'
                break
            elif self.goal_msg.goal.x == 2:
                for count in range(iteration):
                    self.triangle(feedback_msg, goal_handle, length)
                feedback_msg.state = 'triangle patrol complete!!'
                break

        goal_handle.succeed()
        result = Patrol.Result()
        result.result = feedback_msg.state

        self.init_twist()
        self.get_logger().info('Patrol complete.')
        threading.Timer(0.1, rclpy.shutdown).start()

        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 13 * (90.0 / 180.0) * math.pi / 100.0

        for i in range(4):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(90.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self.init_twist()

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 8 * (120.0 / 180.0) * math.pi / 100.0

        for i in range(3):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(120.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_server = Turtlebot3PatrolServer()

    rclpy.spin(turtlebot3_patrol_server)


if __name__ == '__main__':
    main()