#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # TODO: create a ROS publisher for topic 'chatter'
    # pub = rospy.Publisher(...)

    # TODO: initialize node 'talker'
    # rospy.init_node(...)

    rate = rospy.Rate(1)  # Keep this line

    while not rospy.is_shutdown():
        msg = "Hello world %s" % rospy.get_time()   # Keep message logic
        rospy.loginfo(msg)

        # TODO: publish the message
        # pub.publish(...)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
