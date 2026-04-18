#!/usr/bin/env python

"""
Demo: Ask item (HandCamera_Value, Delta)
'ask_item' service send 'Read' request command to controller.
'tm_driver' node receive the result and publish it to 'tm_driver/svr_response'.
('tm_driver' node must be running)

'ask_item' service request param.:
string id -> response 'id' is same as request 'id'
string item -> item_name you want to ask
float64 wait_time ->

If 'wait_time' == 0,
the service call is non-blocking, only send 'Read' request.
The response data is NULL,
but you can still get result from topic 'tm_driver/svr_response'.

If 'wait_time' > 0,
the service call is blocking with timeout 'wait_time' sec. until the result is received.
You can get result in response data.
"""

import rclpy
from rclpy.node import Node
from tm_msgs.srv import AskItem

class AskItemDemo(Node):
    def __init__(self):
        super().__init__('ask_item_demo')
        self.cli = self.create_client(AskItem, 'ask_item')

    def ask_item_demo(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ask_item service not available, waiting...')
        req = AskItem.Request()
        req.id = 'id'
        req.item = 'HandCamera_Value'
        req.wait_time = 5.0
        future = self.cli.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Result: id: %s, content: %s' % (response.id, response.content.strip('{}')))
                break

def main(args=None):
    rclpy.init(args=args)
    ask_item_demo = AskItemDemo()
    ask_item_demo.ask_item_demo()
    ask_item_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()