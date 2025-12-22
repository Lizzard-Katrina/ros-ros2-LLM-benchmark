#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

class RGBDPointCloudGenerator:
    def __init__(self):
        # TODO: implement synchronization + point cloud publishing logic
        # END: point cloud generation logic ends here

    def generate_point_cloud(self, rgb_msg, depth_msg):
        rospy.loginfo("Synchronizing RGB and Depth...")

if __name__ == "__main__":
    rospy.init_node("rgbd_pointcloud_generator")
    node = RGBDPointCloudGenerator()
    rospy.spin()
