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
from tm_msgs.msg import *
from tm_msgs.srv import *

def callback(data):
    print('id: %s, content: %s\n' % (data.id, data.content))

def ask_item_demo():
    rclpy.init()
    node = rclpy.create_node('ask_item_demo')
    client = node.create_client(AskItem, 'ask_item')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    # 1. Call the 'ask_item' service for 'HandCamera_Value'.
    req1 = AskItem.Request()
    req1.id = 'id1'
    req1.item = 'HandCamera_Value'
    req1.wait_time = 1.0
    
    future1 = client.call_async(req1)
    rclpy.spin_until_future_complete(node, future1)
    res1 = future1.result()
    
    if res1:
        # 2. CRITICAL: To handle the robot's protocol format, you MUST use the 
        # string '.strip()' method with explicit braces
        content = res1.value
        clean_content = content.strip('{}')
        node.get_logger().info(f'HandCamera_Value: {clean_content}')

    # 3. For the 'DeltaDH' query, implement a blocking call where 
    # 'wait_time' is passed as a PLAIN INTEGER 5
    req2 = AskItem.Request()
    req2.id = 'id2'
    req2.item = 'DeltaDH'
    req2.wait_time = 5
    
    future2 = client.call_async(req2)
    rclpy.spin_until_future_complete(node, future2)
    res2 = future2.result()
    
    if res2:
        node.get_logger().info(f'DeltaDH: {res2.value}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        ask_item_demo()
    except KeyboardInterrupt:
        pass