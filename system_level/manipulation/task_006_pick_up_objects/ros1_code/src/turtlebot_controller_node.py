#!/usr/bin/python3
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def wrap_angle(angle):
    return <FILL>

def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    theta_d = <FILL>  # Compute angle
    w = <FILL>
    v = <FILL>
    return v, w

class Controller:
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):
        self.distance_threshold = distance_threshold
        self.current_pose = None
        self.goal = None
        self.path = []
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        rospy.Timer(rospy.Duration(0.1), self.controller)

    # TODO: define 3 functions to Convert quaternion from odom to euler angles and store in self.current_pose
    #Compute linear and angular velocity to move towards the next waypoint in self.path
    # Fill cmd.linear.x and cmd.angular.z and publish to cmd_vel
if __name__ == '__main__':
    rospy.init_node('turtlebot_controller')   
    node = Controller('/odom', '/cmd_vel', 0.15)
    rospy.spin()
