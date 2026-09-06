#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped

import math


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi))))


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """Computes the control command to move from current position to goal."""
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.05:  # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w


class Controller(Node):
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):
        super().__init__('turtlebot_controller')
        # 1. Store the node and distance_threshold.
        self.distance_threshold = distance_threshold
        self.current_pose = None
        self.goal = None
        self.path = None
        self.Kv = 0.5
        self.Kw = 0.5
        self.v_max = 0.5
        self.w_max = 1.0

        # Strip leading slashes for ROS 2 convention
        odom_topic = odom_topic.lstrip('/')
        cmd_vel_topic = cmd_vel_topic.lstrip('/')

        # 2. Create a Publisher for 'cmd_vel_topic' (Twist, qos=10).
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'path_marker', 10)

        # 3. Create a Subscriber for 'odom_topic' (Odometry) and 'move_base_simple/goal' (PoseStamped).
        self.create_subscription(Odometry, odom_topic, self.get_odom, 10)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.get_goal, 10)

        # 5. Create a Timer (10Hz) to call 'self.controller'.
        self.create_timer(0.1, self.controller)

    def get_odom(self, odom):
        _, _, yaw = euler_from_quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            self.get_logger().info("New goal received: ({}, {})".format(
                goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None
            self.path = [self.current_pose[0:2], self.goal]
            self.publish_path(self.path)
            del self.path[0]

    # Iterate
    def controller(self):
        v = 0
        w = 0
        if self.path is not None and len(self.path) > 0:
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                self.get_logger().info("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    self.get_logger().info("Final position reached!")
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
            self.get_logger().info("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
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
    node = Controller('odom', 'cmd_vel', 0.15)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()