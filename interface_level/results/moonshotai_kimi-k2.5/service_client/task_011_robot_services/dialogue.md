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
from sys import exit


class CameraReconfigure(object):
    def __init__(self, node):
        self.node = node
        self.param_client = node.create_client(SetParameters, '/camera/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for camera parameter service...')
    
    def disable_auto(self):
        self._set_auto(False)
    
    def enable_auto(self):
        self._set_auto(True)
    
    def _set_auto(self, enabled):
        req = SetParameters.Request()
        
        auto_exposure = Parameter()
        auto_exposure.name = 'auto_exposure'
        auto_exposure.value = ParameterValue(type=ParameterValue.BOOL, bool_value=enabled)
        
        auto_white_balance = Parameter()
        auto_white_balance.name = 'auto_white_balance'
        auto_white_balance.value = ParameterValue(type=ParameterValue.BOOL, bool_value=enabled)
        
        req.parameters = [auto_exposure, auto_white_balance]
        
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            for result in future.result().results:
                if not result.successful:
                    self.node.get_logger().error(f'Failed to set parameter: {result.reason}')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rclpy.init()
    node = rclpy.create_node("camera_reconfigure")
    reconfigure = CameraReconfigure(node)

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    import time
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()