#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
# image_transport import
# TODO: import image_transport Publisher

def main():
    rospy.init_node('camera_publisher_node')
    
    # TODO: 使用 image_transport 创建 publisher
    # pub = ...

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: 填充 Image 消息
        img_msg = Image()
        # pub.publish(img_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
