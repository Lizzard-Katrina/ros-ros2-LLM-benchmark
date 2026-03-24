#!/usr/bin/env python

# Copyright (C) 2015 Fetch Robotics Inc
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

# Author: Michael Ferguson

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter


class CameraReconfigure(Node):
    def __init__(self):
        super().__init__('camera_reconfigure')
        self.cli = self.create_client(GetParameters, 'set_camera_info')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()
        self.req.names = ['auto_exposure', 'auto_white_balance']

    def disable_auto(self):
        self.req = SetParameters.Request()
        self.req.parameters = [
            Parameter(name='auto_exposure', value=False),
            Parameter(name='auto_white_balance', value=False)
        ]
        self.future = self.cli.call_async(self.req)

    def enable_auto(self):
        self.req = SetParameters.Request()
        self.req.parameters = [
            Parameter(name='auto_exposure', value=True),
            Parameter(name='auto_white_balance', value=True)
        ]
        self.future = self.cli.call_async(self.req)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rclpy.init()
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    try:
        rclpy.spin(reconfigure)
    except KeyboardInterrupt:
        reconfigure.get_logger().info('Keyboard Interrupt (SIGINT)')
    except ExternalShutdownException:
        reconfigure.get_logger().info('External Shutdown')
    finally:
        reconfigure.destroy_node()
        rclpy.try_shutdown()