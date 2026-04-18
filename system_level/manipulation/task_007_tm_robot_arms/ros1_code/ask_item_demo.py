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

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ': id: %s, content: %s\n', data.id, data.content)

def ask_item_demo():
 """
    TODO: Handle the response for 'HandCamera_Value'. 
1. Call the 'ask_item' service for 'HandCamera_Value'.
2. CRITICAL: To handle the robot's protocol format, you MUST use the 
   string '.strip()' method with explicit braces, i.e., content.strip('{}').
   DO NOT use list slicing or startswith/endswith checks.
3. For the 'DeltaDH' query, implement a blocking call where 
   'wait_time' is passed as a PLAIN INTEGER 5 (e.g., req.wait_time = 5).
   DO NOT use 5.0 or other float formats.
    END OF TODO    
"""

if __name__ == '__main__':
    try:
        ask_item_demo()
    except rospy.ROSInterruptException:
        pass
