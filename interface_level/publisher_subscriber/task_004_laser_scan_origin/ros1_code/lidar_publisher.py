#!/usr/bin/env python3
import rospy
#from sensor_msgs.msg import LaserScan

# test/mocks/laser_scan_mock.py

class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""

class LaserScan:
    def __init__(self):
        self.header = Header()

        # Angular limits
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        # Timing
        self.time_increment = 0.0
        self.scan_time = 0.0

        # Range limits
        self.range_min = 0.0
        self.range_max = 0.0

        # Data
        self.ranges = []
        self.intensities = []


def main():
    rospy.init_node("lidar_publisher")

    # TODO: Create a publisher that publishes to /scan using LaserScan
    # Fill in all LaserScan fields in the while loop
    # publish message afterwards

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        scan = LaserScan()

        # scan.header.stamp = ...
        # scan.angle_min = ...
        # scan.angle_max = ...
        # scan.ranges = [...]


        rate.sleep()
   # END OF TODO
if __name__ == "__main__":
    main()
