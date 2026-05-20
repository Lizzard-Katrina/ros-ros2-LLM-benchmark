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

[FILENAME: husky_high_level_controller.cpp]
```cpp
#include "husky_high_level_controller/husky_high_level_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

namespace husky_high_level_controller {

HuskyHighLevelController::HuskyHighLevelController()
: Node("husky_high_level_controller")
{
    // Initialize parameters
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 0.5);
    this->declare_parameter("min_distance", 1.0);
    this->declare_parameter("p_gain", 1.0);

    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    // Create subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        std::bind(&HuskyHighLevelController::scanCallback, this, std::placeholders::_1));

    // Create publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Husky High Level Controller started");
}

HuskyHighLevelController::~HuskyHighLevelController()
{
}

void HuskyHighLevelController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Find the closest point in the laser scan
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
        RCLCPP_WARN(this->get_logger(), "No valid laser scan data");
        return;
    }

    // Calculate angle to the closest point
    float angle = msg->angle_min + min_index * msg->angle_increment;

    // Get parameters
    double linear_speed = this->get_parameter("linear_speed").as_double();
    double angular_speed = this->get_parameter("angular_speed").as_double();
    double min_dist_threshold = this->get_parameter("min_distance").as_double();
    double p_gain = this->get_parameter("p_gain").as_double();

    // Create twist message
    auto cmd_vel = geometry_msgs::msg::Twist();

    // Obstacle avoidance logic
    if (min_distance < min_dist_threshold) {
        // Obstacle detected: turn away
        cmd_vel.linear.x = 0.0;
        // Turn away from obstacle based on its angle
        if (angle > 0) {
            cmd_vel.angular.z = -angular_speed;
        } else {
            cmd_vel.angular.z = angular_speed;
        }
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at distance: %.2f, angle: %.2f", min_distance, angle);
    } else {
        // No obstacle: move forward with P-controller to keep centered
        cmd_vel.linear.x = linear_speed;
        // P-controller: steer away from closest point
        cmd_vel.angular.z = -p_gain * angle;
    }

    // Publish command
    cmd_vel_pub_->publish(cmd_vel);
}

} // namespace husky_high_level_controller

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<husky_high_level_controller::HuskyHighLevelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

[FILENAME: husky_high_level_controller.hpp]
```cpp
#ifndef HUSKY_HIGH_LEVEL_CONTROLLER_HPP
#define HUSKY_HIGH_LEVEL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace husky_high_level_controller {

class HuskyHighLevelController : public rclcpp::Node
{
public:
    HuskyHighLevelController();
    ~HuskyHighLevelController();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

} // namespace husky_high_level_controller

#endif // HUSKY_HIGH_LEVEL_CONTROLLER_HPP
```

[FILENAME: CMakeLists.txt]
```cmake
cmake_minimum_required(VERSION 3.5)
project(husky_high_level_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the executable
add_executable(husky_high_level_controller_node
  src/husky_high_level_controller.cpp
)

# Link dependencies
ament_target_dependencies(husky_high_level_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install
install(TARGETS
  husky_high_level_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

[FILENAME: package.xml]
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_high_level_controller</name>
  <version>0.0.0</version>
  <description>ROS2 Husky High Level Controller with obstacle avoidance</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

[FILENAME: launch/husky_high_level_controller.launch.py]
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='husky_high_level_controller',
            executable='husky_high_level_controller_node',
            name='husky_high_level_controller',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'linear_speed': 0.5,
                'angular_speed': 0.5,
                'min_distance': 1.0,
                'p_gain': 1.0
            }]
        )
    ])
```