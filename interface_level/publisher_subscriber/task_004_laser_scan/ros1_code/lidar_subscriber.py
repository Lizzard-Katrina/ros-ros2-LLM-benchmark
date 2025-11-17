#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # TODO: Process the incoming LaserScan message
    # Example: print the closest range value
    # print(min(msg.ranges))
    pass

def main():
    rospy.init_node("lidar_subscriber")

    # TODO: Create a subscriber for /scan topic using LaserScan
    # rospy.Subscriber("...", LaserScan, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
