#!/usr/bin/env python3
import rospy
# mock Image class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def main():
    rospy.init_node('camera_publisher_node')
    
    # TODO: use image_transport to construct publisher
    # and insert information of Image

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    # END OF TODO
if __name__ == '__main__':
    main()
