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
from rclpy.logging import get_logger
from beginner_tutorials.srv import AddTwoInts

def handle_add_two_ints(request, response):
    # TODO: AI/user completes service logic
    get_logger("add_two_ints_server").info("Server received request: %s + %s" % (request.a, request.b))
    response.sum = request.a + request.b
    return response
    # END OF TODO

def server_node():
    rclpy.init()
    node = rclpy.create_node('add_two_ints_server')
    # TODO: advertise the service
    srv = node.create_service(AddTwoInts, 'add_two_ints', handle_add_two_ints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # END OF TODO


if __name__ == "__main__":
    server_node()