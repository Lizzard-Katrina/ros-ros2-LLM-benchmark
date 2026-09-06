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
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, ParameterType


class CameraReconfigure(object):

    def __init__(self):
        self.node = rclpy.create_node('camera_reconfigure')
        self.parameter_client = self.node.create_client(
            SetParameters,
            '/head_camera/driver/set_parameters'
        )
        self.node.get_logger().info('Waiting for parameter service on head_camera/driver...')
        self.parameter_client.wait_for_service(timeout_sec=5.0)

    def disable_auto(self):
        request = SetParameters.Request()
        auto_exposure_param = ParameterMsg()
        auto_exposure_param.name = 'auto_exposure'
        auto_exposure_param.value = ParameterValue()
        auto_exposure_param.value.type = ParameterType.PARAMETER_BOOL
        auto_exposure_param.value.bool_value = False

        auto_white_balance_param = ParameterMsg()
        auto_white_balance_param.name = 'auto_white_balance'
        auto_white_balance_param.value = ParameterValue()
        auto_white_balance_param.value.type = ParameterType.PARAMETER_BOOL
        auto_white_balance_param.value.bool_value = False

        request.parameters = [auto_exposure_param, auto_white_balance_param]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.node.get_logger().info('Disabled auto_exposure and auto_white_balance')

    def enable_auto(self):
        request = SetParameters.Request()
        auto_exposure_param = ParameterMsg()
        auto_exposure_param.name = 'auto_exposure'
        auto_exposure_param.value = ParameterValue()
        auto_exposure_param.value.type = ParameterType.PARAMETER_BOOL
        auto_exposure_param.value.bool_value = True

        auto_white_balance_param = ParameterMsg()
        auto_white_balance_param.name = 'auto_white_balance'
        auto_white_balance_param.value = ParameterValue()
        auto_white_balance_param.value.type = ParameterType.PARAMETER_BOOL
        auto_white_balance_param.value.bool_value = True

        request.parameters = [auto_exposure_param, auto_white_balance_param]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.node.get_logger().info('Enabled auto_exposure and auto_white_balance')

    def destroy(self):
        self.node.destroy_node()


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        sys.exit(-1)

    reconfigure = CameraReconfigure()

    try:
        if sys.argv[1] == "--enable":
            reconfigure.enable_auto()
        else:
            reconfigure.disable_auto()
    finally:
        reconfigure.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()