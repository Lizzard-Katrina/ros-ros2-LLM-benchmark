#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node('robot2_publisher')
    pub = rospy.Publisher('_____', LaserScan, queue_size=10)
    rate = rospy.Rate(_____)  # frequency in Hz

    while not rospy.is_shutdown():
        msg = LaserScan()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
