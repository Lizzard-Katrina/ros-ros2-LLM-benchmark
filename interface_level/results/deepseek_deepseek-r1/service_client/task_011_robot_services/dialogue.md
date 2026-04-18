# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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
import rospy
import dynamic_reconfigure.client
from sys import exit


class CameraReconfigure(object):
# TODO:define methods of this class that
# Initialize a ROS2-side interface
# Disable automatic exposure and white balance
# Enable automatic exposure and white balance
# END OF TODO


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rospy.init_node("camera_reconfigure")
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    rospy.sleep(1)

----------------------------


# LLM Response

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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rclpy.parameter import ParameterType
import time


class CameraReconfigure(Node):
    def __init__(self, camera_node_name):
        super().__init__('camera_reconfigure')
        self.camera_node_name = camera_node_name
        self.param_client = self.create_client(
            SetParameters,
            f'/{camera_node_name}/set_parameters'
        )
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def set_parameter(self, name, value, type):
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=type)
        if type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = value
        elif type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = value
        elif type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = value
        elif type == ParameterType.PARAMETER_STRING:
            param.value.string_value = value
        
        req = SetParameters.Request()
        req.parameters = [param]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def disable_auto(self):
        self.set_parameter('exposure_auto', False, ParameterType.PARAMETER_BOOL)
        self.set_parameter('white_balance_auto', False, ParameterType.PARAMETER_BOOL)
        self.get_logger().info('Disabled auto exposure and white balance')

    def enable_auto(self):
        self.set_parameter('exposure_auto', True, ParameterType.PARAMETER_BOOL)
        self.set_parameter('white_balance_auto', True, ParameterType.PARAMETER_BOOL)
        self.get_logger().info('Enabled auto exposure and white balance')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable [camera_node_name]")
        exit(-1)

    rclpy.init()
    
    camera_node = "camera"
    if len(sys.argv) == 3:
        camera_node = sys.argv[2]
    
    reconfigure = CameraReconfigure(camera_node)

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    time.sleep(1)
    rclpy.shutdown()