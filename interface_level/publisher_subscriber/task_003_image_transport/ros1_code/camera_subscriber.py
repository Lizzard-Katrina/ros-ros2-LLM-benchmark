#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
# image_transport import
# TODO: import image_transport Subscriber

def callback(msg):
    # TODO: 检测点：确保消息接收使用了 image_transport
    rospy.loginfo("Received an image")

def main():
    rospy.init_node('camera_subscriber_node')
    
    # TODO: 使用 image_transport 创建 subscriber
    # sub = ...

    rospy.spin()

if __name__ == '__main__':
    main()
