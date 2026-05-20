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

install(TARGETS husky_highlevel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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

  std::string subscriber_topic_;
  std::string publisher_topic_;
  int queue_size_;
  float p_gain_;
};

} // namespace husky_highlevel_controller

#endif // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController()
: Node("husky_highlevel_controller")
{
  this->declare_parameter<std::string>("subscriber_topic", "/scan");
  this->declare_parameter<std::string>("publisher_topic", "/cmd_vel");
  this->declare_parameter<int>("queue_size", 10);
  this->declare_parameter<float>("p_gain", 1.0);

  this->get_parameter("subscriber_topic", subscriber_topic_);
  this->get_parameter("publisher_topic", publisher_topic_);
  this->get_parameter("queue_size", queue_size_);
  this->get_parameter("p_gain", p_gain_);

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    subscriber_topic_, queue_size_,
    std::bind(&HuskyHighlevelController::laserScanCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(publisher_topic_, queue_size_);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", queue_size_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

void HuskyHighlevelController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  float min_distance = std::numeric_limits<float>::max();
  int min_index = -1;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    if (range >= msg->range_min && range <= msg->range_max) {
      if (range < min_distance) {
        min_distance = range;
        min_index = i;
      }
    }
  }

  if (min_index < 0) {
    RCLCPP_WARN(this->get_logger(), "No valid range found in laser scan.");
    return;
  }

  float angle = msg->angle_min + min_index * msg->angle_increment;
  float x = min_distance * std::cos(angle);
  float y = min_distance * std::sin(angle);

  RCLCPP_INFO(this->get_logger(), "Closest pillar at: x=%f, y=%f, distance=%f", x, y, min_distance);

  publishMarker(x, y);
  publishTransform(x, y);

  geometry_msgs::msg::Twist cmd_vel;
  float angular_error = -angle;
  cmd_vel.angular.z = p_gain_ * angular_error;

  float forward_speed = 0.0;
  if (min_distance > 1.0) {
    forward_speed = 0.5 * (min_distance - 1.0);
    forward_speed = std::min(forward_speed, 1.0f);
  }
  cmd_vel.linear.x = forward_speed;

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
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  marker_pub_->publish(marker);
}

void HuskyHighlevelController::publishTransform(float x, float y) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "base_laser";
  transformStamped.child_frame_id = "pillar";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transformStamped);
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