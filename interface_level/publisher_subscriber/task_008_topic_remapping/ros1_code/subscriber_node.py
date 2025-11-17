#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Received: %s", msg.data)

def main():
    rospy.init_node('subscriber_node')

    # TODO: topic name may be remapped via launch
    rospy.Subscriber('_____', String, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
