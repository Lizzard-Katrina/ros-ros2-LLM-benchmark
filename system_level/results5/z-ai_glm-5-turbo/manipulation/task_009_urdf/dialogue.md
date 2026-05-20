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
project(husky_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(husky_base src/husky_base.cpp)
ament_target_dependencies(husky_base rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros std_msgs)

install(TARGETS husky_base
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

[FILENAME: package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_base</name>
  <version>0.6.0</version>
  <description>Husky base controller</description>
  <maintainer email="todo@todo.com">Husky Maintainer</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

[FILENAME: src/husky_base.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class HuskyBase : public rclcpp::Node
{
public:
  HuskyBase() : Node("husky_base")
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&HuskyBase::cmdVelCallback, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&HuskyBase::update, this));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
  }

  void update()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    double delta_x = (linear_velocity_ * cos(theta_) - 0.0 * sin(theta_)) * dt;
    double delta_y = (linear_velocity_ * sin(theta_) + 0.0 * cos(theta_)) * dt;
    double delta_theta = angular_velocity_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta_));

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.angular.z = angular_velocity_;

    odom_pub_->publish(odom);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = current_time;
    joint_state.name = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
    joint_state.position = {0.0, 0.0, 0.0, 0.0};
    joint_state.velocity = {linear_velocity_ / 0.1, linear_velocity_ / 0.1, linear_velocity_ / 0.1, linear_velocity_ / 0.1};
    joint_state_pub_->publish(joint_state);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(odom_tf);

    last_time_ = current_time;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;
  double x_;
  double y_;
  double theta_;
  rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HuskyBase>());
  rclcpp::shutdown();
  return 0;
}
]