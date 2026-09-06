#!/usr/bin/env python3
"""
A Python node that implements the same kinematics and sonar logic
as turtle.cpp, for runtime verification purposes.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist


def normalize_angle(angle):
    TWO_PI = 2.0 * math.pi
    PI = math.pi
    return angle - (TWO_PI * math.floor((angle + PI) / TWO_PI))


def compute_sonar_distance(pos_x, pos_y, orient, canvas_width, canvas_height):
    """
    Compute sonar distance using the same logic as turtle.cpp.
    30-degree FOV, analytical intersection, epsilon guards, max range, y-mirroring.
    """
    NUM_RAYS = 11
    FOV = math.radians(30.0)
    MAX_RANGE = 5.0
    sonar_distance_ = MAX_RANGE

    for i in range(NUM_RAYS):
        angle = orient - FOV / 2.0 + (FOV * i) / (NUM_RAYS - 1)
        dx = math.cos(angle)
        dy = -math.sin(angle)

        min_dist = MAX_RANGE

        # Right wall (canvas_width)
        if abs(dx) > 1e-6:
            t = (canvas_width - pos_x) / dx
            if t > 0 and t < min_dist:
                min_dist = t
            # Left wall (0)
            t = (0.0 - pos_x) / dx
            if t > 0 and t < min_dist:
                min_dist = t

        # Bottom wall (canvas_height)
        if abs(dy) > 1e-6:
            t = (canvas_height - pos_y) / dy
            if t > 0 and t < min_dist:
                min_dist = t
            # Top wall (0)
            t = (0.0 - pos_y) / dy
            if t > 0 and t < min_dist:
                min_dist = t

        if min_dist < sonar_distance_:
            sonar_distance_ = min_dist

    return sonar_distance_


def update_pose(pos_x, pos_y, orient, lin_vel_x, lin_vel_y, ang_vel, dt,
                canvas_width, canvas_height, holonomic=False):
    """
    Update pose using the same logic as turtle.cpp.
    Returns (new_x, new_y, new_orient, sonar_distance, hit_wall)
    """
    orient += ang_vel * dt
    orient = normalize_angle(orient)

    # Holonomic kinematics with rotation matrix
    pos_x += (math.cos(orient) * lin_vel_x - math.sin(orient) * lin_vel_y) * dt
    pos_y += (-math.sin(orient) * lin_vel_x - math.cos(orient) * lin_vel_y) * dt

    hit_wall = False
    if pos_x < 0.0 or pos_x > canvas_width or pos_y < 0.0 or pos_y > canvas_height:
        hit_wall = True

    pos_x = max(0.0, min(pos_x, canvas_width))
    pos_y = max(0.0, min(pos_y, canvas_height))

    sonar_dist = compute_sonar_distance(pos_x, pos_y, orient, canvas_width, canvas_height)

    return pos_x, pos_y, orient, sonar_dist, hit_wall


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        self.canvas_width = 11.0
        self.canvas_height = 11.0
        self.pos_x = 5.5
        self.pos_y = 5.5
        self.orient = 0.0
        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel = 0.0
        self.holonomic = False

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)

        self.pose_pub = self.create_publisher(Float64MultiArray, 'turtle_pose', 10)
        self.sonar_pub = self.create_publisher(Float64MultiArray, 'sonar_distance', 10)

        self.timer = self.create_timer(0.016, self.update_callback)
        self.get_logger().info('Kinematics node started')

    def cmd_callback(self, msg):
        self.lin_vel_x = msg.linear.x
        self.lin_vel_y = msg.linear.y
        self.ang_vel = msg.angular.z

    def update_callback(self):
        dt = 0.016
        self.pos_x, self.pos_y, self.orient, sonar_dist, hit_wall = update_pose(
            self.pos_x, self.pos_y, self.orient,
            self.lin_vel_x, self.lin_vel_y, self.ang_vel, dt,
            self.canvas_width, self.canvas_height, self.holonomic)

        pose_msg = Float64MultiArray()
        pose_msg.data = [self.pos_x, self.canvas_height - self.pos_y, self.orient]
        self.pose_pub.publish(pose_msg)

        sonar_msg = Float64MultiArray()
        sonar_msg.data = [sonar_dist]
        self.sonar_pub.publish(sonar_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()