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
cmake_minimum_required(VERSION 3.5)
project(husky_highlevel_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace husky_highlevel_controller {

class HuskyHighlevelController : public rclcpp::Node {
public:
  HuskyHighlevelController();
  virtual ~HuskyHighlevelController();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishMarker(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index);
  void publishTransform(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index);
  void driveToPillar(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double p_gain_;
  double x_min_;
  double x_max_;
  double pillar_diameter_;
};

} // namespace husky_highlevel_controller

#endif // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController()
: Node("husky_highlevel_controller")
{
  // Declare parameters
  this->declare_parameter("p_gain", 1.0);
  this->declare_parameter("x_min", -1.0);
  this->declare_parameter("x_max", 1.0);
  this->declare_parameter("pillar_diameter", 0.3);

  // Get parameters
  p_gain_ = this->get_parameter("p_gain").as_double();
  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  pillar_diameter_ = this->get_parameter("pillar_diameter").as_double();

  // Create subscriber
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    std::bind(&HuskyHighlevelController::laserScanCallback, this, std::placeholders::_1));

  // Create publisher for velocity commands
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Create publisher for marker visualization
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

  // Create TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(this->get_logger(), "HuskyHighlevelController initialized");
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

void HuskyHighlevelController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Find the closest point in the laser scan within the x range
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();

  double angle_min = msg->angle_min;
  double angle_increment = msg->angle_increment;

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    double range = msg->ranges[i];
    if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
      double angle = angle_min + i * angle_increment;
      double x = range * std::cos(angle);
      if (x >= x_min_ && x <= x_max_) {
        if (range < closest_distance) {
          closest_distance = range;
          closest_index = i;
        }
      }
    }
  }

  if (closest_distance < std::numeric_limits<double>::max()) {
    RCLCPP_DEBUG(this->get_logger(), "Closest pillar at index %zu, distance %f", closest_index, closest_distance);
    publishMarker(msg, closest_index);
    publishTransform(msg, closest_index);
    driveToPillar(msg, closest_index);
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid pillar detected in scan");
    // Stop the robot if no pillar is detected
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
  }
}

void HuskyHighlevelController::publishMarker(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = msg->header.frame_id;
  marker.header.stamp = this->now();
  marker.ns = "pillar";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  double angle = msg->angle_min + index * msg->angle_increment;
  double range = msg->ranges[index];
  marker.pose.position.x = range * std::cos(angle);
  marker.pose.position.y = range * std::sin(angle);
  marker.pose.position.z = 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = pillar_diameter_;
  marker.scale.y = pillar_diameter_;
  marker.scale.z = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker_pub_->publish(marker);
}

void HuskyHighlevelController::publishTransform(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = msg->header.frame_id;
  transform_stamped.child_frame_id = "pillar";

  double angle = msg->angle_min + index * msg->angle_increment;
  double range = msg->ranges[index];
  transform_stamped.transform.translation.x = range * std::cos(angle);
  transform_stamped.transform.translation.y = range * std::sin(angle);
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = 0.0;
  transform_stamped.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transform_stamped);
}

void HuskyHighlevelController::driveToPillar(const sensor_msgs::msg::LaserScan::SharedPtr &msg, std::size_t index) {
  double angle = msg->angle_min + index * msg->angle_increment;
  double range = msg->ranges[index];

  // Calculate the angular error (angle to the pillar)
  double angular_error = angle;

  // Calculate the linear error (distance to the pillar minus a desired distance)
  double desired_distance = 1.0; // Desired distance to keep from the pillar
  double linear_error = range - desired_distance;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = p_gain_ * linear_error;
  cmd.angular.z = p_gain_ * angular_error;

  // Clamp the velocities to reasonable limits
  double max_linear_speed = 0.5;
  double max_angular_speed = 1.0;
  cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_speed, max_linear_speed);
  cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_speed, max_angular_speed);

  cmd_vel_pub_->publish(cmd);
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
                {'x_min': -1.0},
                {'x_max': 1.0},
                {'pillar_diameter': 0.3}
            ]
        )
    ])