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
cmake_minimum_required(VERSION 3.8)
project(husky_high_level_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(husky_high_level_controller src/husky_high_level_controller.cpp src/husky_high_level_controller_node.cpp)
ament_target_dependencies(husky_high_level_controller rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS husky_high_level_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()

[FILENAME: package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_high_level_controller</name>
  <version>0.1.0</version>
  <description>High-level controller for the Husky robot.</description>
  <maintainer email="ros-developer@example.com">ROS Developer</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

[FILENAME: src/husky_high_level_controller.cpp]
#include "husky_high_level_controller/husky_high_level_controller.hpp"

namespace husky_high_level_controller {

HuskyHighLevelController::HuskyHighLevelController(const rclcpp::NodeOptions & options)
    : Node("husky_high_level_controller", options),
      target_pillar_x_(0.0),
      target_pillar_y_(0.0),
      kp_angular_(0.5),
      kp_linear_(0.5),
      max_linear_velocity_(0.5),
      max_angular_velocity_(1.0)
{
    RCLCPP_INFO(this->get_logger(), "HuskyHighLevelController node has been started.");

    // Declare parameters
    this->declare_parameter<double>("kp_angular", kp_angular_);
    this->declare_parameter<double>("kp_linear", kp_linear_);
    this->declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    this->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

    // Get parameters
    this->get_parameter("kp_angular", kp_angular_);
    this->get_parameter("kp_linear", kp_linear_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a subscriber for laser scan messages
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&HuskyHighLevelController::scanCallback, this, std::placeholders::_1));

    // Create a publisher for velocity commands
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
}

void HuskyHighLevelController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Find the closest pillar
    float min_distance = msg->range_max;
    int min_angle_index = -1;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < min_distance) {
            min_distance = msg->ranges[i];
            min_angle_index = i;
        }
    }

    if (min_angle_index != -1) {
        // Calculate the angle to the closest pillar
        float angle_to_pillar = msg->angle_min + min_angle_index * msg->angle_increment;

        // Transform the pillar's polar coordinates to Cartesian coordinates in the laser frame
        target_pillar_x_ = min_distance * cos(angle_to_pillar);
        target_pillar_y_ = min_distance * sin(angle_to_pillar);

        // Transform the pillar's coordinates from the laser frame to the base_link frame
        geometry_msgs::msg::PointStamped laser_point;
        laser_point.header.frame_id = msg->header.frame_id;
        laser_point.header.stamp = this->get_clock()->now();
        laser_point.point.x = target_pillar_x_;
        laser_point.point.y = target_pillar_y_;
        laser_point.point.z = 0.0;

        geometry_msgs::msg::PointStamped base_link_point;
        try {
            tf_buffer_->transform(laser_point, base_link_point, "base_link", tf2::Duration(std::chrono::seconds(1)));
            target_pillar_x_ = base_link_point.point.x;
            target_pillar_y_ = base_link_point.point.y;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point from %s to base_link: %s",
                        laser_point.header.frame_id.c_str(), ex.what());
            return;
        }

        // Calculate control commands
        double angle_error = atan2(target_pillar_y_, target_pillar_x_);
        double distance_error = sqrt(target_pillar_x_ * target_pillar_x_ + target_pillar_y_ * target_pillar_y_);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = kp_angular_ * angle_error;
        cmd_vel_msg.linear.x = kp_linear_ * (distance_error - 0.5); // Maintain a distance of 0.5m

        // Clamp velocities
        if (cmd_vel_msg.linear.x > max_linear_velocity_) {
            cmd_vel_msg.linear.x = max_linear_velocity_;
        } else if (cmd_vel_msg.linear.x < -max_linear_velocity_) {
            cmd_vel_msg.linear.x = -max_linear_velocity_;
        }

        if (cmd_vel_msg.angular.z > max_angular_velocity_) {
            cmd_vel_msg.angular.z = max_angular_velocity_;
        } else if (cmd_vel_msg.angular.z < -max_angular_velocity_) {
            cmd_vel_msg.angular.z = -max_angular_velocity_;
        }

        // Publish velocity commands
        velocity_publisher_->publish(cmd_vel_msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "No pillar detected in scan.");
        // Stop the robot if no pillar is detected
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        velocity_publisher_->publish(cmd_vel_msg);
    }
}

} // namespace husky_high_level_controller

[FILENAME: src/husky_high_level_controller_node.cpp]
#include "rclcpp/rclcpp.hpp"
#include "husky_high_level_controller/husky_high_level_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<husky_high_level_controller::HuskyHighLevelController>());
    rclcpp::shutdown();
    return 0;
}

[FILENAME: include/husky_high_level_controller/husky_high_level_controller.hpp]
#ifndef HUSKY_HIGH_LEVEL_CONTROLLER_HUSKY_HIGH_LEVEL_CONTROLLER_HPP
#define HUSKY_HIGH_LEVEL_CONTROLLER_HUSKY_HIGH_LEVEL_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace husky_high_level_controller {

class HuskyHighLevelController : public rclcpp::Node
{
public:
    HuskyHighLevelController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double target_pillar_x_;
    double target_pillar_y_;

    double kp_angular_;
    double kp_linear_;
    double max_linear_velocity_;
    double max_angular_velocity_;
};

} // namespace husky_high_level_controller

#endif // HUSKY_HIGH_LEVEL_CONTROLLER_HUSKY_HIGH_LEVEL_CONTROLLER_HPP