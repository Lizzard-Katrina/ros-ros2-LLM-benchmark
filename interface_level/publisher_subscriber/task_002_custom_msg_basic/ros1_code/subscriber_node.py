#!/usr/bin/env python3
import rospy
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def callback(msg):
    # TODO
    # Print received data
    rospy.loginfo(f"Received: {msg}")

def main():
    rospy.init_node('person_subscriber')

    # Create a subscriber listening to /person_info

    rospy.spin()
    #END OF TODO
if __name__ == '__main__':
    main()
