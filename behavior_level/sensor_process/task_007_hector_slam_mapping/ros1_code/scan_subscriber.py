#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    rospy.loginfo("Received scan with {} ranges".format(len(msg.ranges)))

def main():
    rospy.init_node("scan_listener")

    # TODO: create a subscriber to /scan using LaserScan messages
    # END: subscriber creation ends here

    rospy.spin()

if __name__ == "__main__":
    main()
