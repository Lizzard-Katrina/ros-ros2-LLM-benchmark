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
project(husky_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(husky_msgs REQUIRED)

include_directories(
  include
)

add_executable(husky_node
  src/husky_base.cpp
  src/husky_hardware.cpp
  src/husky_diagnostics.cpp
  src/horizon_legacy/crc.cpp
  src/horizon_legacy/Logger.cpp
  src/horizon_legacy/Message.cpp
  src/horizon_legacy/Message_cmd.cpp
  src/horizon_legacy/Message_data.cpp
  src/horizon_legacy/Message_request.cpp
  src/horizon_legacy/Number.cpp
  src/horizon_legacy/Transport.cpp
  src/horizon_legacy/linux_serial.cpp
)

ament_target_dependencies(husky_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  diagnostic_updater
  diagnostic_msgs
  husky_msgs
)

install(TARGETS husky_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

[FILENAME: include/husky_base/husky_hardware.h]
#ifndef HUSKY_BASE_HUSKY_HARDWARE_H
#define HUSKY_BASE_HUSKY_HARDWARE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "husky_base/husky_diagnostics.h"

#include <string>
#include <vector>
#include <cmath>

namespace husky_base
{

class HuskyHardware
{
public:
  HuskyHardware(rclcpp::Node::SharedPtr node);
  ~HuskyHardware();

  void reportLoopDuration(const rclcpp::Duration &duration);

  void updateJointsFromHardware();

  void writeCommandsToHardware();

  void publishJointState();

  void publishOdometry();

private:
  void resetTravelOffset();

  void initializeDiagnostics();

  double linearToAngular(const double &travel) const;

  double angularToLinear(const double &angle) const;

  void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  husky_base::HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus> system_status_task_;
  husky_base::HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem> power_status_task_;
  husky_base::HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus> safety_status_task_;
  husky_base::HuskySoftwareDiagnosticTask software_status_task_;

  struct Joint
  {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), position_offset(0), velocity(0), effort(0), velocity_command(0) {}
  } joints_[4];

  std::string joint_names_[4];

  double wheel_diameter_;
  double max_accel_;
  double max_speed_;
  double polling_timeout_;

  geometry_msgs::msg::Twist cmd_vel_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_HARDWARE_H

[FILENAME: include/husky_base/husky_diagnostics.h]
#ifndef HUSKY_BASE_HUSKY_DIAGNOSTICS_H
#define HUSKY_BASE_HUSKY_DIAGNOSTICS_H

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>

#include "husky_base/horizon_legacy/Message.h"
#include "husky_base/horizon_legacy/Message_data.h"

namespace husky_base
{

template<typename T>
class HuskyHardwareDiagnosticTask : public diagnostic_updater::DiagnosticTask
{
public:
  HuskyHardwareDiagnosticTask(const std::string &name)
    : DiagnosticTask(name)
  {
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
  {
    typename T::Ptr msg = T::getUpdate();
    if (msg)
    {
      update(stat, msg);
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data received.");
    }
  }

  void update(diagnostic_updater::DiagnosticStatusWrapper &stat, typename T::Ptr msg);
};

class HuskySoftwareDiagnosticTask : public diagnostic_updater::DiagnosticTask
{
public:
  HuskySoftwareDiagnosticTask(const std::string &name, rclcpp::Clock::SharedPtr clock)
    : DiagnosticTask(name), clock_(clock)
  {
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

  void updateControlFrequency(double frequency);

private:
  rclcpp::Clock::SharedPtr clock_;
  double control_freq_ = 0.0;
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_DIAGNOSTICS_H

[FILENAME: src/husky_base.cpp]
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "husky_base/husky_hardware.h"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("husky_node");

  husky_base::HuskyHardware husky(node);

  auto last_time = node->get_clock()->now();

  double target_control_freq;
  node->declare_parameter<double>("control_frequency", 10.0);
  node->get_parameter("control_frequency", target_control_freq);

  rclcpp::Rate rate(target_control_freq);

  while (rclcpp::ok())
  {
    auto current_time = node->get_clock()->now();
    auto duration = current_time - last_time;
    last_time = current_time;

    husky.updateJointsFromHardware();
    husky.publishJointState();
    husky.publishOdometry();
    husky.writeCommandsToHardware();
    husky.reportLoopDuration(duration);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

[FILENAME: src/husky_hardware.cpp]
#include "husky_base/husky_hardware.h"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "husky_base/horizon_legacy/horizon_legacy.h"

namespace husky_base
{

HuskyHardware::HuskyHardware(rclcpp::Node::SharedPtr node)
  : node_(node),
    system_status_task_("System Status"),
    power_status_task_("Power Status"),
    safety_status_task_("Safety Status"),
    software_status_task_("Software Status", node->get_clock())
{
  node_->declare_parameter<double>("wheel_diameter", 0.3302);
  node_->declare_parameter<double>("max_accel", 5.0);
  node_->declare_parameter<double>("max_speed", 1.0);
  node_->declare_parameter<double>("polling_timeout", 10.0);
  node_->declare_parameter<std::string>("port", "/dev/prolific");

  node_->get_parameter("wheel_diameter", wheel_diameter_);
  node_->get_parameter("max_accel", max_accel_);
  node_->get_parameter("max_speed", max_speed_);
  node_->get_parameter("polling_timeout", polling_timeout_);

  std::string port;
  node_->get_parameter("port", port);

  joint_names_[0] = "front_left_wheel";
  joint_names_[1] = "front_right_wheel";
  joint_names_[2] = "rear_left_wheel";
  joint_names_[3] = "rear_right_wheel";

  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_ = *msg;
    });

  horizon_legacy::connect(port);
  horizon_legacy::configureLimits(max_speed_, max_accel_);
  resetTravelOffset();
  initializeDiagnostics();
}

HuskyHardware::~HuskyHardware()
{
}

void HuskyHardware::reportLoopDuration(const rclcpp::Duration &duration)
{
  software_status_task_.updateControlFrequency(1.0 / duration.seconds());
  diagnostic_updater_->force_update();
}

void HuskyHardware::updateJointsFromHardware()
{
  clearpath::DataEncoders::Ptr enc = clearpath::DataEncoders::getUpdate(polling_timeout_);
  if (enc)
  {
    for (int i = 0; i < 4; i++)
    {
      double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;
      if (std::abs(delta) > M_PI)
      {
        delta = (delta > 0) ? delta - 2.0 * M_PI : delta + 2.0 * M_PI;
      }
      joints_[i].position += delta;
    }

    clearpath::DataDifferentialSpeed::Ptr speed = clearpath::DataDifferentialSpeed::getUpdate(polling_timeout_);
    if (speed)
    {
      joints_[0].velocity = linearToAngular(speed->getLeftSpeed());
      joints_[1].velocity = linearToAngular(speed->getRightSpeed());
      joints_[2].velocity = joints_[0].velocity;
      joints_[3].velocity = joints_[1].velocity;
    }
  }
}

void HuskyHardware::writeCommandsToHardware()
{
  double diff_speed_left = angularToLinear(joints_[0].velocity_command);
  double diff_speed_right = angularToLinear(joints_[1].velocity_command);

  double left_cmd = cmd_vel_.linear.x - cmd_vel_.angular.z * 0.555 / 2.0;
  double right_cmd = cmd_vel_.linear.x + cmd_vel_.angular.z * 0.555 / 2.0;

  limitDifferentialSpeed(left_cmd, right_cmd);

  horizon_legacy::controlSpeed(left_cmd, right_cmd, max_accel_, max_accel_);
}

void HuskyHardware::publishJointState()
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->get_clock()->now();

  for (int i = 0; i < 4; i++)
  {
    joint_state.name.push_back(joint_names_[i]);
    joint_state.position.push_back(joints_[i].position);
    joint_state.velocity.push_back(joints_[i].velocity);
    joint_state.effort.push_back(joints_[i].effort);
  }

  joint_state_pub_->publish(joint_state);
}

void HuskyHardware::publishOdometry()
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = node_->get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // Simplified odometry - in a real system this would integrate wheel encoders
  odom.twist.twist.linear.x = (joints_[0].velocity + joints_[1].velocity) * wheel_diameter_ / 4.0;
  odom.twist.twist.angular.z = (joints_[1].velocity - joints_[0].velocity) * wheel_diameter_ / (2.0 * 0.555);

  odom_pub_->publish(odom);
}

void HuskyHardware::resetTravelOffset()
{
  clearpath::DataEncoders::Ptr enc = clearpath::DataEncoders::getUpdate(polling_timeout_);
  if (enc)
  {
    for (int i = 0; i < 4; i++)
    {
      joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
    }
  }
}

void HuskyHardware::initializeDiagnostics()
{
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
  diagnostic_updater_->setHardwareID("Husky");
  diagnostic_updater_->add(system_status_task_);
  diagnostic_updater_->add(power_status_task_);
  diagnostic_updater_->add(safety_status_task_);
  diagnostic_updater_->add(software_status_task_);
}

double HuskyHardware::linearToAngular(const double &travel) const
{
  return travel / wheel_diameter_ * 2.0;
}

double HuskyHardware::angularToLinear(const double &angle) const
{
  return angle * wheel_diameter_ / 2.0;
}

void HuskyHardware::limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right)
{
  double large_speed = std::max(std::abs(travel_speed_left), std::abs(travel_speed_right));
  if (large_speed > max_speed_)
  {
    travel_speed_left *= max_speed_ / large_speed;
    travel_speed_right *= max_speed_ / large_speed;
  }
}

}  // namespace husky_base

[FILENAME: src/husky_diagnostics.cpp]
#include "husky_base/husky_diagnostics.h"

#include <string>

namespace husky_base
{

template<>
void HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus>::update(
  diagnostic_updater::DiagnosticStatusWrapper &stat,
  clearpath::DataSystemStatus::Ptr msg)
{
  stat.add("Uptime", msg->getUptime());
  stat.add("Battery Voltage", msg->getVoltage(0));
  stat.add("Temperature", msg->getTemperature(0));

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "System Status OK");
}

template<>
void HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem>::update(
  diagnostic_updater::DiagnosticStatusWrapper &stat,
  clearpath::DataPowerSystem::Ptr msg)
{
  stat.add("Battery Charge", msg->getCharge(0));
  stat.add("Battery Capacity", msg->getCapacity(0));

  double charge_percent = 0.0;
  if (msg->getCapacity(0) != 0)
  {
    charge_percent = msg->getCharge(0) / msg->getCapacity(0) * 100.0;
  }

  if (charge_percent > 50.0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery OK");
  }
  else if (charge_percent > 20.0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery Low");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery Critical");
  }
}

template<>
void HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
  diagnostic_updater::DiagnosticStatusWrapper &stat,
  clearpath::DataSafetySystemStatus::Ptr msg)
{
  uint16_t flags = msg->getFlags();
  stat.add("Safety Flags", flags);

  if (flags == 0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Safety System OK");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Safety flags active");
  }
}

void HuskySoftwareDiagnosticTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Control Frequency (Hz)", control_freq_);

  if (control_freq_ > 8.0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Control loop running");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Control loop slow");
  }
}

void HuskySoftwareDiagnosticTask::updateControlFrequency(double frequency)
{
  control_freq_ = frequency;
}

}  // namespace husky_base

[FILENAME: package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>husky_base</name>
  <version>0.5.0</version>
  <description>Husky base ROS2 driver</description>
  <maintainer email="admin@clearpathrobotics.com">Clearpath Robotics</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>diagnostic_updater</depend>
  <depend>diagnostic_msgs</depend>
  <depend>husky_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>