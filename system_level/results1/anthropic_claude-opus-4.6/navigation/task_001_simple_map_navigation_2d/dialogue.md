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



[FILENAME: CMakeLists.txt]
cmake_minimum_required(VERSION 3.5)
project(husky_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(husky_odom_listener src/husky_odom_listener.cpp)
ament_target_dependencies(husky_odom_listener
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  husky_odom_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/husky_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

[FILENAME: package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_example</name>
  <version>0.0.1</version>
  <description>Husky robot example package for ROS2</description>
  <maintainer email="admin@example.com">admin</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

[FILENAME: src/husky_odom_listener.cpp]
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class HuskyOdomListener : public rclcpp::Node
{
public:
  HuskyOdomListener() : Node("husky_odom_listener")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "husky_velocity_controller/odom", 10,
      std::bind(&HuskyOdomListener::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Husky Odom Listener started, waiting for odometry data...");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double linear_vel = msg->twist.twist.linear.x;
    double angular_vel = msg->twist.twist.angular.z;

    RCLCPP_INFO(this->get_logger(),
      "Position: (%.3f, %.3f, %.3f) | Yaw: %.3f | Linear Vel: %.3f | Angular Vel: %.3f",
      x, y, z, yaw, linear_vel, angular_vel);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HuskyOdomListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

[FILENAME: scripts/husky_teleop.py]
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

MSG = """
Husky Teleop Control
--------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class HuskyTeleop(Node):
    def __init__(self):
        super().__init__('husky_teleop')
        self.publisher_ = self.create_publisher(Twist, 'husky_velocity_controller/cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.get_logger().info('Husky Teleop node started')

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(MSG)
        print(f"currently:\tspeed {self.speed}\tturn {self.turn}")

        try:
            while rclpy.ok():
                key = get_key(settings)
                if key in MOVE_BINDINGS:
                    x = MOVE_BINDINGS[key][0]
                    th = MOVE_BINDINGS[key][1]
                elif key in SPEED_BINDINGS:
                    self.speed *= SPEED_BINDINGS[key][0]
                    self.turn *= SPEED_BINDINGS[key][1]
                    print(f"currently:\tspeed {self.speed:.2f}\tturn {self.turn:.2f}")
                    x = 0
                    th = 0
                elif key == 'k':
                    x = 0
                    th = 0
                elif key == '\x03':
                    break
                else:
                    x = 0
                    th = 0

                twist = Twist()
                twist.linear.x = float(x) * self.speed
                twist.angular.z = float(th) * self.turn
                self.publisher_.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')

        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = HuskyTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

[FILENAME: launch/husky_example.launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='husky_example',
            executable='husky_odom_listener',
            name='husky_odom_listener',
            output='screen',
        ),
        Node(
            package='husky_example',
            executable='husky_teleop.py',
            name='husky_teleop',
            output='screen',
            prefix='xterm -e',
        ),
    ])