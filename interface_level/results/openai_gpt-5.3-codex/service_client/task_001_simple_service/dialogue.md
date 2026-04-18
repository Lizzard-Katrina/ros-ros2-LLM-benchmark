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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from beginner_tutorials.srv import AddTwoInts

_node = None


def handle_add_two_ints(req, response):
    # TODO: AI/user completes service logic
    _node.get_logger().info(f"Server received request: {req.a} + {req.b}")
    response.sum = req.a + req.b
    return response
    # END OF TODO


def server_node():
    rclpy.init()
    global _node
    _node = rclpy.create_node('add_two_ints_server')
    # TODO: advertise the service
    _node.create_service(AddTwoInts, 'add_two_ints', handle_add_two_ints)
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()
    # END OF TODO


if __name__ == "__main__":
    server_node()