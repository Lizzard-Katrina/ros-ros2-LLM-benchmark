# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from your_package.srv import AddThreeInts, AddThreeIntsResponse

def handle_add_three_ints(req):
    # TODO: fill computation
    rospy.loginfo("Received request: %s %s %s", req.a, req.b, req.c)
    # return correct response
    return  
    # END OF TODO

def server_node():
    rospy.init_node("add_three_ints_server")
    # TODO: advertise service
    rospy.loginfo("Custom service server started.")
    rospy.spin()
    # END OF TODO
if __name__ == "__main__":
    server_node()

----------------------------


# LLM Response

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from your_package.srv import AddThreeInts

class AddThreeIntsServer(Node):
    def __init__(self):
        super().__init__('add_three_ints_server')
        # TODO: advertise service
        self.srv = self.create_service(
            AddThreeInts, 'add_three_ints', self.handle_add_three_ints)
        self.get_logger().info("Custom service server started.")
        # END OF TODO

    def handle_add_three_ints(self, request, response):
        # TODO: fill computation
        self.get_logger().info(f"Received request: {request.a} {request.b} {request.c}")
        response.sum = request.a + request.b + request.c
        return response
        # END OF TODO

def server_node():
    rclpy.init()
    server = AddThreeIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == "__main__":
    server_node()
```