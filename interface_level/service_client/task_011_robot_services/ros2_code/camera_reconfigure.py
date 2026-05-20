#!/usr/bin/env python3

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
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sys import exit


class CameraReconfigure(Node):
    def __init__(self):
        super().__init__('camera_reconfigure')
        self.client = self.create_client(SetParameters, '/camera/camera_node/set_parameters')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Camera parameter service not available')

    def _set_auto_params(self, enable: bool):
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable)
        req.parameters.append(Parameter(name='auto_exposure', value=val))
        req.parameters.append(Parameter(name='auto_white_balance', value=val))
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def disable_auto(self):
        self.get_logger().info('Disabling auto exposure and white balance...')
        self._set_auto_params(False)

    def enable_auto(self):
        self.get_logger().info('Enabling auto exposure and white balance...')
        self._set_auto_params(True)


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

    time.sleep(1)
    
    reconfigure.destroy_node()
    rclpy.shutdown()