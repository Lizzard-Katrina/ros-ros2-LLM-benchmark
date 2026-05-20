# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):


# LLM Response

[FILENAME: husky_high_level_controller.py]
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

class HuskyHighLevelController(Node):
    def __init__(self):
        super().__init__('husky_high_level_controller')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('obstacle_distance_threshold', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.scan_ranges = []
        self.state = 'MOVING_TO_GOAL'  # States: MOVING_TO_GOAL, AVOIDING_OBSTACLE, GOAL_REACHED
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Husky High Level Controller started')
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
    
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
    
    def get_distance_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        desired_yaw = math.atan2(dy, dx)
        angle_diff = desired_yaw - self.current_yaw
        # Normalize angle to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        return angle_diff
    
    def check_obstacle_in_front(self):
        if not self.scan_ranges:
            return False
        
        # Check front sector (roughly -30 to +30 degrees)
        front_ranges = []
        num_ranges = len(self.scan_ranges)
        # Assuming 360 degree scan, front is around index 0 and end
        front_indices = list(range(0, 30)) + list(range(num_ranges-30, num_ranges))
        
        for i in front_indices:
            if i < num_ranges and self.scan_ranges[i] > 0.0:
                front_ranges.append(self.scan_ranges[i])
        
        if front_ranges:
            min_front = min(front_ranges)
            return min_front < self.obstacle_threshold
        return False
    
    def check_obstacle_left(self):
        if not self.scan_ranges:
            return False
        
        num_ranges = len(self.scan_ranges)
        # Left sector (roughly 30 to 90 degrees)
        left_indices = list(range(30, 90))
        left_ranges = []
        
        for i in left_indices:
            if i < num_ranges and self.scan_ranges[i] > 0.0:
                left_ranges.append(self.scan_ranges[i])
        
        if left_ranges:
            min_left = min(left_ranges)
            return min_left < self.obstacle_threshold
        return False
    
    def check_obstacle_right(self):
        if not self.scan_ranges:
            return False
        
        num_ranges = len(self.scan_ranges)
        # Right sector (roughly -90 to -30 degrees)
        right_indices = list(range(num_ranges-90, num_ranges-30))
        right_ranges = []
        
        for i in right_indices:
            if i < num_ranges and self.scan_ranges[i] > 0.0:
                right_ranges.append(self.scan_ranges[i])
        
        if right_ranges:
            min_right = min(right_ranges)
            return min_right < self.obstacle_threshold
        return False
    
    def control_loop(self):
        if not self.odom_received:
            self.get_logger().warn('Waiting for odometry...')
            return
        
        distance_to_goal = self.get_distance_to_goal()
        
        # Check if goal reached
        if distance_to_goal < self.goal_tolerance:
            if self.state != 'GOAL_REACHED':
                self.get_logger().info('Goal reached!')
                self.state = 'GOAL_REACHED'
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
            return
        
        # State machine
        if self.state == 'MOVING_TO_GOAL':
            if self.check_obstacle_in_front():
                self.get_logger().info('Obstacle detected! Switching to avoidance.')
                self.state = 'AVOIDING_OBSTACLE'
            else:
                self.move_to_goal()
        
        elif self.state == 'AVOIDING_OBSTACLE':
            if not self.check_obstacle_in_front():
                self.get_logger().info('Path clear. Resuming to goal.')
                self.state = 'MOVING_TO_GOAL'
            else:
                self.avoid_obstacle()
    
    def move_to_goal(self):
        cmd = Twist()
        angle_to_goal = self.get_angle_to_goal()
        
        if abs(angle_to_goal) > 0.1:
            # Rotate towards goal
            cmd.angular.z = self.angular_speed if angle_to_goal > 0 else -self.angular_speed
            cmd.linear.x = 0.0
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def avoid_obstacle(self):
        cmd = Twist()
        
        if self.check_obstacle_left() and not self.check_obstacle_right():
            # Turn right
            cmd.angular.z = -self.angular_speed
            cmd.linear.x = 0.0
        elif self.check_obstacle_right() and not self.check_obstacle_left():
            # Turn left
            cmd.angular.z = self.angular_speed
            cmd.linear.x = 0.0
        else:
            # Both blocked or both free, turn right as default
            cmd.angular.z = -self.angular_speed
            cmd.linear.x = 0.0
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = HuskyHighLevelController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()