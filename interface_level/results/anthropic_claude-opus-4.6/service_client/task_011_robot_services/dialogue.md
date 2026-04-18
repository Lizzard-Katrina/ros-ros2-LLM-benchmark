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



```python
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
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


class CameraReconfigure(object):

    def __init__(self, node):
        self.node = node
        self.client = self.node.create_client(
            SetParameters,
            '/head_camera/driver/set_parameters'
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for parameter service...')

    def disable_auto(self):
        params = []

        p_exposure = Parameter()
        p_exposure.name = 'auto_exposure'
        p_exposure.value = ParameterValue()
        p_exposure.value.type = ParameterType.PARAMETER_BOOL
        p_exposure.value.bool_value = False
        params.append(p_exposure)

        p_white_balance = Parameter()
        p_white_balance.name = 'auto_white_balance'
        p_white_balance.value = ParameterValue()
        p_white_balance.value.type = ParameterType.PARAMETER_BOOL
        p_white_balance.value.bool_value = False
        params.append(p_white_balance)

        request = SetParameters.Request()
        request.parameters = params
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Disabled automatic exposure and white balance')

    def enable_auto(self):
        params = []

        p_exposure = Parameter()
        p_exposure.name = 'auto_exposure'
        p_exposure.value = ParameterValue()
        p_exposure.value.type = ParameterType.PARAMETER_BOOL
        p_exposure.value.bool_value = True
        params.append(p_exposure)

        p_white_balance = Parameter()
        p_white_balance.name = 'auto_white_balance'
        p_white_balance.value = ParameterValue()
        p_white_balance.value.type = ParameterType.PARAMETER_BOOL
        p_white_balance.value.bool_value = True
        params.append(p_white_balance)

        request = SetParameters.Request()
        request.parameters = params
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Enabled automatic exposure and white balance')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        sys.exit(-1)

    rclpy.init(args=sys.argv)
    node = Node("camera_reconfigure")
    reconfigure = CameraReconfigure(node)

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    import time
    time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()
```