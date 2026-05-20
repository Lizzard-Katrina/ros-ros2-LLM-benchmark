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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace husky_highlevel_controller {

class HuskyHighlevelController : public rclcpp::Node {
public:
  HuskyHighlevelController();
  virtual ~HuskyHighlevelController();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishMarker(float x, float y);
  void publishTransform(float x, float y);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  float p_gain_;
  float min_distance_;
};

} // namespace husky_highlevel_controller

#endif // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController()
: Node("husky_highlevel_controller_node") {

  this->declare_parameter<float>("p_gain", 1.0);
  this->declare_parameter<float>("min_distance", 1.0);

  this->get_parameter("p_gain", p_gain_);
  this->get_parameter("min_distance", min_distance_);

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&HuskyHighlevelController::laserScanCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

void HuskyHighlevelController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges.empty()) {
    return;
  }

  float min_range = std::numeric_limits<float>::infinity();
  int min_index = -1;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    if (std::isfinite(range) && range < min_range) {
      min_range = range;
      min_index = static_cast<int>(i);
    }
  }

  if (min_index < 0) {
    return;
  }

  float angle = msg->angle_min + min_index * msg->angle_increment;
  float x = min_range * std::cos(angle);
  float y = min_range * std::sin(angle);

  publishMarker(x, y);
  publishTransform(x, y);

  geometry_msgs::msg::Twist cmd_vel;
  float error = min_range - min_distance_;
  float angular_correction = -angle * p_gain_;

  cmd_vel.linear.x = std::max(0.0, error * p_gain_);
  cmd_vel.angular.z = angular_correction;

  cmd_vel_pub_->publish(cmd_vel);
}

void HuskyHighlevelController::publishMarker(float x, float y) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_laser";
  marker.header.stamp = this->now();
  marker.ns = "pillar";
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
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker_pub_->publish(marker);
}

void HuskyHighlevelController::publishTransform(float x, float y) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = "base_laser";
  transform_stamped.child_frame_id = "pillar";
  transform_stamped.transform.translation.x = x;
  transform_stamped.transform.translation.y = y;
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

} // namespace husky_highlevel_controller

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
            parameters=[
                {'p_gain': 1.0},
                {'min_distance': 1.0}
            ]
        )
    ])