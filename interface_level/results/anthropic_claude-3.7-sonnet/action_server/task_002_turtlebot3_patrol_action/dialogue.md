# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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

from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print('TurtleBot3 Patrol Server')
        print('----------------------------------------------')

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
        while True:
            position += self.twist.linear.x
            if position >= length:
                break
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(1)
        self.init_twist()

    def turn(self, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            current_yaw = self.get_yaw()
            yaw_diff = abs(
                math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw)
                )
            )

            if yaw_diff < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # TODO:
        # 1. Accept the patrol goal and decide which patrol pattern to execute.
        # 2. Perform the patrol according to the goal specifications.
        # 3. During the patrol, continuously provide feedback about the patrol progress.
        # 4. After completing the patrol, mark the goal as succeeded and provide a result summarizing the patrol outcome.
        # END OF TODO
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

----------------------------


# LLM Response

```python
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

from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print('TurtleBot3 Patrol Server')
        print('----------------------------------------------')

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
        while True:
            position += self.twist.linear.x
            if position >= length:
                break
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(1)
        self.init_twist()

    def turn(self, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            current_yaw = self.get_yaw()
            yaw_diff = abs(
                math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw)
                )
            )

            if yaw_diff < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()
        
        # Accept the patrol goal and decide which patrol pattern to execute
        patrol_type = self.goal_msg.patrol_type
        radius = self.goal_msg.radius
        
        # Perform the patrol according to the goal specifications
        if patrol_type == 1:  # Square patrol
            self.get_logger().info('Executing square patrol')
            self.square(feedback_msg, goal_handle, radius)
            result.success = True
        elif patrol_type == 2:  # Triangle patrol
            self.get_logger().info('Executing triangle patrol')
            self.triangle(feedback_msg, goal_handle, radius)
            result.success = True
        else:
            self.get_logger().error('Invalid patrol type')
            goal_handle.abort()
            result.success = False
            return result
        
        # After completing the patrol, mark the goal as succeeded
        goal_handle.succeed()
        
        # Provide a result summarizing the patrol outcome
        result.message = f"Completed patrol type {patrol_type} with radius {radius}"
        
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
```