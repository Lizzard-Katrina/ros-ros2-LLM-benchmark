#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    # TODO: initialize ROS1 node
    # TODO: create publisher for /chatter topic

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello from ros1"
        # TODO: publish the message
        rate.sleep()

if __name__ == "__main__":
    main()
