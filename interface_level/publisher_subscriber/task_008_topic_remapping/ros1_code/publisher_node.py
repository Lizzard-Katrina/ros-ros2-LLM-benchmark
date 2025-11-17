#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('publisher_node')

    # TODO: topic name may be remapped via launch
    pub = rospy.Publisher('_____', String, queue_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hello, remapping test!"
        pub.publish(msg)
        rospy.loginfo("Published: %s", msg.data)
        rate.sleep()

if __name__ == "__main__":
    main()
