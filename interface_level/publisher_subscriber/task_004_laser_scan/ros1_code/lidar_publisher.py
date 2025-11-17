#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node("lidar_publisher")

    # TODO: Create a publisher that publishes to /scan using LaserScan
    # pub = rospy.Publisher("...", LaserScan, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        scan = LaserScan()

        # TODO: Fill in LaserScan fields:
        # scan.header.stamp = ...
        # scan.angle_min = ...
        # scan.angle_max = ...
        # scan.ranges = [...]

        # TODO: Publish LaserScan message
        # pub.publish(scan)

        rate.sleep()

if __name__ == "__main__":
    main()
