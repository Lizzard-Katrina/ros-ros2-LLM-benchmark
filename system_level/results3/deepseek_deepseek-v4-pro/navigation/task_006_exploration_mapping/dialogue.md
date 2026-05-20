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

add_executable(husky_highlevel_controller
  src/husky_highlevel_controller.cpp
)

target_include_directories(husky_highlevel_controller
  PRIVATE
  include
)

ament_target_dependencies(husky_highlevel_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS
  husky_highlevel_controller
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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class HuskyHighlevelController : public rclcpp::Node
{
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

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string laser_topic_;
  std::string cmd_vel_topic_;
  std::string marker_topic_;
  std::string base_frame_;
  std::string odom_frame_;

  double p_gain_linear_;
  double p_gain_angular_;
};

#endif // HUSKY_HIGHLEVEL_CONTROLLER_HPP

[FILENAME: src/husky_highlevel_controller.cpp]
#include "husky_highlevel_controller/husky_highlevel_controller.hpp"
#include <cmath>
#include <algorithm>

HuskyHighlevelController::HuskyHighlevelController()
: Node("husky_highlevel_controller")
{
  // Declare parameters
  this->declare_parameter<std::string>("laser_topic", "/scan");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("marker_topic", "/visualization_marker");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<double>("p_gain_linear", 0.5);
  this->declare_parameter<double>("p_gain_angular", 0.8);

  // Get parameters
  laser_topic_ = this->get_parameter("laser_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  marker_topic_ = this->get_parameter("marker_topic").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  p_gain_linear_ = this->get_parameter("p_gain_linear").as_double();
  p_gain_angular_ = this->get_parameter("p_gain_angular").as_double();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create subscriber
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, 10,
    std::bind(&HuskyHighlevelController::laserScanCallback, this, std::placeholders::_1));

  // Create publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Husky Highlevel Controller started");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Find the closest point (minimum range) in the laser scan
  float min_range = std::numeric_limits<float>::max();
  int min_index = -1;

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    float range = msg->ranges[i];
    // Ignore invalid measurements
    if (std::isfinite(range) && range > msg->range_min && range < msg->range_max)
    {
      if (range < min_range)
      {
        min_range = range;
        min_index = static_cast<int>(i);
      }
    }
  }

  if (min_index < 0)
  {
    RCLCPP_WARN(this->get_logger(), "No valid laser scan points found");
    return;
  }

  // Calculate angle of the closest point
  float angle = msg->angle_min + min_index * msg->angle_increment;

  // Convert polar to Cartesian coordinates
  float x = min_range * std::cos(angle);
  float y = min_range * std::sin(angle);

  RCLCPP_DEBUG(this->get_logger(), "Closest point: range=%f, angle=%f, x=%f, y=%f", min_range, angle, x, y);

  // Publish marker at the closest point position
  publishMarker(x, y);

  // Publish transform for the closest point
  publishTransform(x, y);

  // Control logic: steer towards the pillar (closest point)
  geometry_msgs::msg::Twist cmd_vel;

  // Angular velocity: proportional to the angle error
  cmd_vel.angular.z = -p_gain_angular_ * angle;

  // Linear velocity: proportional to distance error (desired distance = 1.0m)
  double distance_error = min_range - 1.0;
  cmd_vel.linear.x = p_gain_linear_ * distance_error;

  // Limit velocities
  cmd_vel.linear.x = std::max(-0.5, std::min(0.5, cmd_vel.linear.x));
  cmd_vel.angular.z = std::max(-1.0, std::min(1.0, cmd_vel.angular.z));

  cmd_vel_pub_->publish(cmd_vel);
}

void HuskyHighlevelController::publishMarker(float x, float y)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_;
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

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker_pub_->publish(marker);
}

void HuskyHighlevelController::publishTransform(float x, float y)
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = base_frame_;
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HuskyHighlevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

[FILENAME: launch/husky_highlevel_controller.launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value='/scan',
        description='Laser scan topic'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Command velocity topic'
    )

    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='/visualization_marker',
        description='Marker topic'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )

    p_gain_linear_arg = DeclareLaunchArgument(
        'p_gain_linear',
        default_value='0.5',
        description='Linear P-gain'
    )

    p_gain_angular_arg = DeclareLaunchArgument(
        'p_gain_angular',
        default_value='0.8',
        description='Angular P-gain'
    )

    husky_highlevel_controller_node = Node(
        package='husky_highlevel_controller',
        executable='husky_highlevel_controller',
        name='husky_highlevel_controller',
        output='screen',
        parameters=[{
            'laser_topic': LaunchConfiguration('laser_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'p_gain_linear': LaunchConfiguration('p_gain_linear'),
            'p_gain_angular': LaunchConfiguration('p_gain_angular'),
        }]
    )

    return LaunchDescription([
        laser_topic_arg,
        cmd_vel_topic_arg,
        marker_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        p_gain_linear_arg,
        p_gain_angular_arg,
        husky_highlevel_controller_node,
    ])