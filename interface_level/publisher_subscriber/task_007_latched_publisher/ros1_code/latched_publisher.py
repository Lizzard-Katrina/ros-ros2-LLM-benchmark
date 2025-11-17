#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('latched_publisher_node')

    # TODO: fill in latch parameter
    pub = rospy.Publisher('latched_topic', String, queue_size=10, latch=_____)

    msg = String()
    msg.data = "Hello, I am latched!"
    pub.publish(msg)
    rospy.loginfo("Message published. Publisher will now exit.")

if __name__ == "__main__":
    main()
