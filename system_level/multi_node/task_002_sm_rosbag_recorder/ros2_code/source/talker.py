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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

## Simple talker demo that publishes std_msgs/String to a configurable topic

PKG = 'task_002_sm_rosbag_recorder'
NAME = 'talker'

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Support dynamic topic name remapping using the parameter system
        self.declare_parameter('topic_name', 'chatter')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)

        # Implement a 10Hz non-blocking execution model using a Timer
        # Timer period = 0.1 seconds = 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Talker node started, publishing on "{topic_name}"')

    def timer_callback(self):
        # Include a timestamp in the message payload derived from the
        # node's internal synchronized clock (ROS Domain Clock)
        current_time = self.get_clock().now().to_msg()
        time_float = current_time.sec + current_time.nanosec * 1e-9
        msg = String()
        msg.data = f'hello world {time_float}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)


def talker():
    rclpy.init(args=sys.argv)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    talker()