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
  <version>0.0.1</version>
  <description>ROS2 Husky Highlevel Controller</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

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
find_package(tf2_geometry_msgs REQUIRED)

add_executable(husky_highlevel_controller_node
  src/husky_highlevel_controller_node.cpp
  src/husky_highlevel_controller.cpp
)

target_include_directories(husky_highlevel_controller_node
  PUBLIC
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
  tf2_geometry_msgs
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

namespace husky_highlevel_controller {

class HuskyHighlevelController : public rclcpp::Node {
public:
  HuskyHighlevelController();
  virtual ~HuskyHighlevelController();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishMarker(const sensor_msgs::msg::LaserScan::SharedPtr &msg, double closest_angle);
  geometry_msgs::msg::TransformStamped getTransform(
      const std::string &target_frame, const std::string &source_frame);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string robot_base_frame_;
  std::string laser_sensor_frame_;
  double p_gain_;
  double x_velocity_;
};

}  // namespace husky_highlevel_controller

#endif  // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController()
    : Node("husky_highlevel_controller") {
  // Declare and get parameters
  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("laser_sensor_frame", "laser");
  this->declare_parameter<double>("p_gain", 1.0);
  this->declare_parameter<double>("x_velocity", 0.5);

  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  laser_sensor_frame_ = this->get_parameter("laser_sensor_frame").as_string();
  p_gain_ = this->get_parameter("p_gain").as_double();
  x_velocity_ = this->get_parameter("x_velocity").as_double();

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

HuskyHighlevelController::~HuskyHighlevelController() {}

void HuskyHighlevelController::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Find the closest range (minimum distance) in the laser scan
  if (msg->ranges.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty laser scan.");
    return;
  }

  auto min_it = std::min_element(
      msg->ranges.begin(), msg->ranges.end(),
      [](float a, float b) {
        if (std::isnan(a) || std::isinf(a)) return false;
        if (std::isnan(b) || std::isinf(b)) return true;
        return a < b;
      });

  if (min_it == msg->ranges.end() ||
      std::isnan(*min_it) || std::isinf(*min_it)) {
    RCLCPP_WARN(this->get_logger(), "No valid range found in laser scan.");
    return;
  }

  int min_index = std::distance(msg->ranges.begin(), min_it);
  double closest_angle = msg->angle_min + min_index * msg->angle_increment;

  // Publish the marker at the closest point
  publishMarker(msg, closest_angle);

  // Compute angular velocity using P-controller
  double angular_z = p_gain_ * closest_angle;

  // Create and publish Twist message
  geometry_msgs::msg::Twist twist;
  twist.linear.x = x_velocity_;
  twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist);
}

void HuskyHighlevelController::publishMarker(
    const sensor_msgs::msg::LaserScan::SharedPtr &msg, double closest_angle) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = laser_sensor_frame_;
  marker.header.stamp = this->now();
  marker.ns = "closest_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Get the range at the closest angle
  int min_index = static_cast<int>(
      std::round((closest_angle - msg->angle_min) / msg->angle_increment));
  if (min_index < 0 || min_index >= static_cast<int>(msg->ranges.size())) {
    RCLCPP_WARN(this->get_logger(), "Invalid index for marker position.");
    return;
  }
  double range = msg->ranges[min_index];

  // Set position in laser frame
  marker.pose.position.x = range * std::cos(closest_angle);
  marker.pose.position.y = range * std::sin(closest_angle);
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set scale
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set color (red)
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker_pub_->publish(marker);
}

geometry_msgs::msg::TransformStamped HuskyHighlevelController::getTransform(
    const std::string &target_frame, const std::string &source_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
  }
  return transform_stamped;
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
                'p_gain': 1.0,
                'x_velocity': 0.5
            }]
        )
    ])