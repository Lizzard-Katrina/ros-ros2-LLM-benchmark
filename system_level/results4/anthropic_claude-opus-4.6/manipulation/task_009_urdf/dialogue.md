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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(husky_example_node src/husky_example_node.cpp)
ament_target_dependencies(husky_example_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  husky_example_node
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
  <description>Husky robot example for ROS2</description>
  <maintainer email="maintainer@example.com">maintainer</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
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

[FILENAME: src/husky_example_node.cpp]
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

class HuskyExampleNode : public rclcpp::Node
{
public:
  HuskyExampleNode()
  : Node("husky_example_node")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&HuskyExampleNode::laserCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&HuskyExampleNode::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HuskyExampleNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Husky example node started");
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Find the minimum distance in the laser scan
    min_distance_ = std::numeric_limits<float>::max();
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float range = msg->ranges[i];
      if (range > msg->range_min && range < msg->range_max)
      {
        if (range < min_distance_)
        {
          min_distance_ = range;
        }
      }
    }
    laser_received_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    odom_received_ = true;
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!laser_received_)
    {
      // No laser data yet, don't move
      cmd_vel_pub_->publish(cmd);
      return;
    }

    // Simple obstacle avoidance behavior
    if (min_distance_ < 1.0)
    {
      // Obstacle detected, turn in place
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.5;
    }
    else
    {
      // No obstacle, drive forward
      cmd.linear.x = 0.5;
      cmd.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float min_distance_ = std::numeric_limits<float>::max();
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  bool laser_received_ = false;
  bool odom_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HuskyExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

[FILENAME: launch/husky_example.launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='husky_example',
            executable='husky_example_node',
            name='husky_example_node',
            output='screen',
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                ('cmd_vel', '/cmd_vel'),
            ]
        ),
    ])