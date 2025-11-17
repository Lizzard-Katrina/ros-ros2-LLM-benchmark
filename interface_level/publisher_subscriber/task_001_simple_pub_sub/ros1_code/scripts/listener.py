#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    # Keep logging logic
    rospy.loginfo("I heard %s", data.data)

def listener():
    # TODO: initialize node 'listener'
    # rospy.init_node(...)

    # TODO: subscribe to topic 'chatter'
    # rospy.Subscriber(...)

    rospy.spin()  # Keep spin()

if __name__ == '__main__':
    listener()


