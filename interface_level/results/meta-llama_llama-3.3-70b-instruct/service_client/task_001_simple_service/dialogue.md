# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    # TODO: AI/user completes service logic
    rospy.loginfo("Server received request: %s + %s", req.a, req.b)
    return AddTwoIntsResponse()
    # END OF TODO

def server_node():
    rospy.init_node('add_two_ints_server')
    # TODO: advertise the service
    rospy.spin()
    # END OF TODO


if __name__ == "__main__":
    server_node()

----------------------------


# LLM Response

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add_two_ints)

    def handle_add_two_ints(self, req, resp):
        # TODO: AI/user completes service logic
        self.get_logger().info("Server received request: %s + %s" % (req.a, req.b))
        resp.sum = req.a + req.b
        return resp
        # END OF TODO

def server_node():
    rclpy.init()
    server = AddTwoIntsServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    server_node()
```