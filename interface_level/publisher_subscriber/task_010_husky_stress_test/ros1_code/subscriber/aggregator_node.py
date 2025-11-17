#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback_robot1(msg):
    rospy.loginfo("Received from robot1")

def callback_robot2(msg):
    rospy.loginfo("Received from robot2")

def main():
    rospy.init_node('aggregator_node')

    rospy.Subscriber('_____', LaserScan, callback_robot1)
    rospy.Subscriber('_____', LaserScan, callback_robot2)

    rospy.spin()

if __name__ == "__main__":
    main()
