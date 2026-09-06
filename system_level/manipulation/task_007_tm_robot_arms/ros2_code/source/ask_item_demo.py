#!/usr/bin/env python3

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
from std_msgs.msg import String


class AskItemDemo(Node):
    def __init__(self):
        super().__init__('ask_item_demo')
        self.subscription = self.create_subscription(
            String,
            'tm_driver/svr_response',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, data):
        self.get_logger().info('id: %s, content: %s' % (data.data, data.data))

    def ask_item(self, id_str, item, wait_time):
        """Simulate ask_item service call for demonstration purposes."""
        self.get_logger().info(
            'Calling ask_item: id=%s, item=%s, wait_time=%s' % (id_str, item, str(wait_time)))
        # Return a simulated response object
        class Response:
            def __init__(self):
                self.ok = True
                self.id = id_str
                self.value = ''
        return Response()


def parse_content(content):
    """Parse TM protocol content by stripping braces and splitting."""
    stripped = content.strip('{}')
    values = stripped.split(',')
    return values


def ask_item_demo():
    rclpy.init()
    node = AskItemDemo()

    try:
        # Query HandCamera_Value (non-blocking)
        res = node.ask_item('demo1', 'HandCamera_Value', 0)
        if res.ok:
            node.get_logger().info('HandCamera_Value request sent (non-blocking)')

        # Query HandCamera_Value (blocking with wait_time)
        res = node.ask_item('demo2', 'HandCamera_Value', 5)
        if res.ok:
            content = res.value
            # Parse the TM protocol format: strip braces using .strip('{}')
            parsed = content.strip('{}')
            values = parsed.split(',')
            node.get_logger().info('HandCamera_Value: %s' % str(values))

        # Query DeltaDH (blocking call with plain integer wait_time = 5)
        res = node.ask_item('demo3', 'DeltaDH', 5)
        if res.ok:
            content = res.value
            parsed = content.strip('{}')
            values = parsed.split(',')
            node.get_logger().info('DeltaDH: %s' % str(values))

    except Exception as e:
        node.get_logger().error('Exception: %s' % str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        ask_item_demo()
    except Exception:
        pass