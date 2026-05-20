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

[FILENAME: package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_highlevel_controller</name>
  <version>0.0.0</version>
  <description>The husky_highlevel_controller package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

[FILENAME: CMakeLists.txt]
cmake_minimum_required(VERSION 3.8)
project(husky_highlevel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(husky_highlevel_controller_node
  src/husky_highlevel_controller_node.cpp
  src/husky_highlevel_controller.cpp
)

target_include_directories(husky_highlevel_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(husky_highlevel_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS
  husky_highlevel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

[FILENAME: include/husky_highlevel_controller/husky_highlevel_controller.hpp]
#ifndef HUSKY_HIGHLEVEL_CONTROLLER_HPP
#define HUSKY_HIGHLEVEL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace husky_highlevel_controller {

class HuskyHighlevelController : public rclcpp::Node {
public:
  HuskyHighlevelController();
  ~HuskyHighlevelController() = default;

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishMarker(float x, float y);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string robot_base_frame_;
  std::string laser_sensor_frame_;
  float p_gain_angular_;
  float p_gain_linear_;
};

}  // namespace husky_highlevel_controller

#endif  // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController()
    : Node("husky_highlevel_controller") {
  // Declare and get parameters
  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("laser_sensor_frame", "laser");
  this->declare_parameter<float>("p_gain_angular", 1.0);
  this->declare_parameter<float>("p_gain_linear", 0.5);

  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  laser_sensor_frame_ = this->get_parameter("laser_sensor_frame").as_string();
  p_gain_angular_ = this->get_parameter("p_gain_angular").as_double();
  p_gain_linear_ = this->get_parameter("p_gain_linear").as_double();

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscriber for laser scan
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&HuskyHighlevelController::laserScanCallback, this,
                std::placeholders::_1));

  // Create publisher for velocity commands
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Create publisher for visualization marker
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);
}

void HuskyHighlevelController::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Find the closest point (minimum range) in the laser scan
  float min_range = std::numeric_limits<float>::max();
  int min_index = -1;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    // Ignore invalid measurements
    if (std::isfinite(range) && range > msg->range_min &&
        range < msg->range_max) {
      if (range < min_range) {
        min_range = range;
        min_index = static_cast<int>(i);
      }
    }
  }

  if (min_index < 0) {
    RCLCPP_WARN(this->get_logger(), "No valid laser scan points found.");
    return;
  }

  // Calculate angle to the closest point
  float angle = msg->angle_min + min_index * msg->angle_increment;

  // Convert polar coordinates to Cartesian (in laser frame)
  float x_laser = min_range * std::cos(angle);
  float y_laser = min_range * std::sin(angle);

  // Publish marker at the closest point in laser frame
  publishMarker(x_laser, y_laser);

  // Compute control commands using P-controller
  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  // Angular velocity: steer towards the pillar (P-controller on angle)
  cmd_vel_msg.angular.z = p_gain_angular_ * angle;

  // Linear velocity: approach the pillar, but stop if too close
  float target_distance = 1.0;  // Desired distance from pillar in meters
  if (min_range > target_distance) {
    cmd_vel_msg.linear.x =
        p_gain_linear_ * (min_range - target_distance);
  } else {
    cmd_vel_msg.linear.x = 0.0;
  }

  // Limit maximum velocities
  float max_linear_speed = 0.5;
  float max_angular_speed = 1.0;
  cmd_vel_msg.linear.x =
      std::clamp(cmd_vel_msg.linear.x, -max_linear_speed, max_linear_speed);
  cmd_vel_msg.angular.z =
      std::clamp(cmd_vel_msg.angular.z, -max_angular_speed, max_angular_speed);

  cmd_vel_pub_->publish(cmd_vel_msg);
}

void HuskyHighlevelController::publishMarker(float x, float y) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = laser_sensor_frame_;
  marker.header.stamp = this->now();
  marker.ns = "pillar_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 1.0;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker_pub_->publish(marker);
}

}  // namespace husky_highlevel_controller

[FILENAME: src/husky_highlevel_controller_node.cpp]
#include <rclcpp/rclcpp.hpp>
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<husky_highlevel_controller::HuskyHighlevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

[FILENAME: launch/husky_highlevel_controller.launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='husky_highlevel_controller',
            executable='husky_highlevel_controller_node',
            name='husky_highlevel_controller',
            output='screen',
            parameters=[{
                'robot_base_frame': 'base_link',
                'laser_sensor_frame': 'laser',
                'p_gain_angular': 1.0,
                'p_gain_linear': 0.5
            }]
        )
    ])