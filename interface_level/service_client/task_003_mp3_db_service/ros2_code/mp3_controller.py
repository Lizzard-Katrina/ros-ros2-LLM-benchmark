Here is the converted ROS2 code:
```python
#!/usr/bin/env python

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2015, Jan Winkler, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Institute for Artificial Intelligence,
#     Universitaet Bremen, nor the names of its contributors may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from ros_service_examples.srv import Mp3Inventory

class Mp3InventoryClient(Node):
  def __init__(self):
    super().__init__('mp3_inventory_client')
    self.cli = self.create_client(Mp3Inventory, 'mp3_inventory_interaction')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

  def send_request(self):
    req = Mp3Inventory.Request()
    # TODO: Fill in the request data
    future = self.cli.call_async(req)
    while rclpy.ok():
      rclpy.spin_once(self)
      if future.done():
        try:
          response = future.result()
        except Exception as e:
          self.get_logger().info('Service call failed %r' % (e,))
        else:
          self.get_logger().info('Result: %r' % (response,))
        break

def main(args=None):
  rclpy.init(args=args)
  mp3_inventory_client = Mp3InventoryClient()
  mp3_inventory_client.send_request()
  mp3_inventory_client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
```
Note that I've replaced the `rospy` imports with `rclpy` and `ros_service_examples.srv` with `ros_service_examples.srv` (assuming the service definition is in a file named `Mp3Inventory.srv`). I've also replaced the `try`-`except` block with a `try`-`except` block that catches `Exception` instead of `rospy.ServiceException`. Additionally, I've added a `while` loop to wait for the service to become available.