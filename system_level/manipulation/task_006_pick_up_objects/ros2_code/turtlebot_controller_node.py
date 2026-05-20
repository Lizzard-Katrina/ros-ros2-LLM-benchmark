#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """ Computes the control command to move from current position to goal. """
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.05: # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w

class Controller(Node):
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):
        super().__init__('turtlebot_controller')
        self.distance_threshold = distance_threshold
        
        odom_topic = odom_topic.lstrip('/')
        cmd_vel_topic = cmd_vel_topic.lstrip('/')
        
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(Marker, 'path_marker', 10)
        
        self.create_subscription(Odometry, odom_topic, self.get_odom, 10)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.get_goal, 10)
        
        self.timer = self.create_timer(0.1, self.controller)
        
        self.current_pose = None
        self.goal = None
        self.path = None
        self.Kv = 0.5
        self.Kw = 0.5
        self.v_max = 0.2
        self.w_max = 1.0

    def get_odom(self, odom):
        _, _, yaw = euler_from_quaternion(odom.pose.pose.orientation.x, 
                                          odom.pose.pose.orientation.y,
                                          odom.pose.pose.orientation.z,
                                          odom.pose.pose.orientation.w)
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2], self.goal]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self):
        v = 0   
        w = 0
        if self.path is not None and len(self.path) > 0 and self.current_pose is not None:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.v_max, self.v_max))
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(np.clip(w, -self.w_max, self.w_max))
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            
            color_red = ColorRGBA()
            color_red.r = 1.0
            color_red.g = 0.0
            color_red.b = 0.0
            color_red.a = 1.0
            color_blue = ColorRGBA()
            color_blue.r = 0.0
            color_blue.g = 0.0
            color_blue.b = 1.0
            color_blue.a = 1.0

            p = Point()
            p.x = float(self.current_pose[0])
            p.y = float(self.current_pose[1])
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = float(n[0])
                p.y = float(n[1])
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Controller('/odom', '/cmd_vel', 0.15)
    
    # Run forever
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()