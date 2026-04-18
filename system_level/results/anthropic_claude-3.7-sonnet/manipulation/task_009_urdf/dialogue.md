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

I'll convert these ROS1 Husky robot files to ROS2, filling in the missing code at TODO locations while maintaining the existing structure and functionality.

[FILENAME: husky_base/include/husky_base/husky_hardware.h]
/**
 *  \file
 *  \brief  ROS hardware interface for Husky
 *  \author Clearpath Robotics Inc.
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef HUSKY_BASE_HUSKY_HARDWARE_H
#define HUSKY_BASE_HUSKY_HARDWARE_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "husky_base/horizon_legacy_wrapper.h"

namespace husky_base
{

class HuskyHardware : public hardware_interface::SystemInterface
{
public:
  HuskyHardware();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void resetTravelOffset();
  void registerControlInterfaces();
  double linearToAngular(const double &travel) const;
  double angularToLinear(const double &angle) const;
  void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

  std::string port_;
  double wheel_diameter_, max_accel_, max_speed_;
  double polling_timeout_;

  /**
  * Joint structure that is hooked to ros_control
  */
  struct Joint
  {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() :
      position(0), position_offset(0), velocity(0), effort(0), velocity_command(0)
    { }
  } joints_[4];

  /**
  * ROS Controller manager and related
  */
  bool initialized_;
  bool use_velocity_commands_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_HARDWARE_H

[FILENAME: husky_base/src/husky_hardware.cpp]
/**
 *  \file
 *  \brief  ROS hardware interface for Husky
 *  \author Clearpath Robotics Inc.
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
#include <vector>
#include <stdexcept>

#include "husky_base/husky_hardware.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace husky_base
{

HuskyHardware::HuskyHardware() :
  initialized_(false),
  use_velocity_commands_(true)
{
}

hardware_interface::CallbackReturn HuskyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize member variables
  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);
  port_ = info_.hardware_parameters["port"];
  polling_timeout_ = std::stod(info_.hardware_parameters["polling_timeout"]);

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Port: %s", port_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Wheel diameter: %f", wheel_diameter_);
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Max accel: %f", max_accel_);
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Max speed: %f", max_speed_);
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Polling timeout: %f", polling_timeout_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize the Horizon legacy wrapper
  horizon_legacy::connect(port_);
  horizon_legacy::configureLimits(max_speed_, max_accel_);
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware configured successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Setup joint state interfaces for the 4 wheels
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HuskyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Setup joint command interfaces for the 4 wheels
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity_command));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HuskyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset joint states
  for (auto i = 0u; i < 4; i++) {
    joints_[i].position = 0;
    joints_[i].position_offset = 0;
    joints_[i].velocity = 0;
    joints_[i].effort = 0;
    joints_[i].velocity_command = 0;
  }
  
  // Reset the encoders and start the platform
  resetTravelOffset();
  horizon_legacy::enableMotors();
  initialized_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  initialized_ = false;
  horizon_legacy::disableMotors();
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware deactivated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!initialized_) {
    return hardware_interface::return_type::OK;
  }

  // Read from robot hardware
  horizon_legacy::getData(polling_timeout_);

  // Populate joint information
  double wheel_angles[4];
  double wheel_velocities[4];

  // Get wheel angles and velocities from the hardware driver
  horizon_legacy::getWheelAngles(wheel_angles);
  horizon_legacy::getWheelVelocities(wheel_velocities);

  for (int i = 0; i < 4; i++) {
    // Update joint information
    joints_[i].position = linearToAngular(wheel_angles[i]);
    joints_[i].velocity = linearToAngular(wheel_velocities[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!initialized_) {
    return hardware_interface::return_type::OK;
  }

  // Calculate wheel velocities
  double wheel_velocity_left = angularToLinear(joints_[0].velocity_command);
  double wheel_velocity_right = angularToLinear(joints_[2].velocity_command);

  // Apply speed limiters
  limitDifferentialSpeed(wheel_velocity_left, wheel_velocity_right);

  // Set wheel speeds
  horizon_legacy::controlSpeed(wheel_velocity_left, wheel_velocity_right);

  return hardware_interface::return_type::OK;
}

void HuskyHardware::resetTravelOffset()
{
  for (int i = 0; i < 4; i++) {
    joints_[i].position_offset = linearToAngular(horizon_legacy::getWheelDistance(i));
  }
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
  double speed = std::max(std::abs(travel_speed_left), std::abs(travel_speed_right));
  if (speed > max_speed_) {
    travel_speed_left = travel_speed_left * max_speed_ / speed;
    travel_speed_right = travel_speed_right * max_speed_ / speed;
  }
}

}  // namespace husky_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(husky_base::HuskyHardware, hardware_interface::SystemInterface)

[FILENAME: husky_base/src/horizon_legacy_wrapper.cpp]
/**
 *  \file
 *  \brief  C++ interface to Husky's Horizon embedded controller
 *  \author  Clearpath Robotics Inc.
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "husky_base/horizon_legacy_wrapper.h"
#include "husky_base/horizon_legacy/clearpath.h"
#include "husky_base/horizon_legacy/message_runtime.h"

namespace horizon_legacy
{

double clamp(double value, double min, double max)
{
  return std::min(std::max(min, value), max);
}

void connect(const std::string& port)
{
  int result = clearpath::Transport::instance().configure(port.c_str());
  if (result != 0)
  {
    throw std::runtime_error("Could not connect to Husky on port: " + port);
  }
  RCLCPP_INFO(rclcpp::get_logger("horizon_legacy"), "Connected to Husky on port %s", port.c_str());
}

void configureLimits(double max_speed, double max_accel)
{
  clearpath::SetMaxAccel accel_message;
  accel_message.setAccel(max_accel);
  accel_message.send();

  clearpath::SetMaxSpeed speed_message;
  speed_message.setSpeed(max_speed);
  speed_message.send();
}

void controlSpeed(double left_speed, double right_speed)
{
  clearpath::SetDifferentialSpeed message;
  message.setLeftSpeed(left_speed);
  message.setRightSpeed(right_speed);
  message.send();
}

void controlDistance(double left_distance, double right_distance)
{
  clearpath::SetDifferentialControl message;
  message.setLeftDistance(left_distance);
  message.setRightDistance(right_distance);
  message.send();
}

void getData(double timeout_sec)
{
  clearpath::DataPlatform& platform = clearpath::DataPlatform::instance();
  platform.receive(timeout_sec);
}

void enableMotors()
{
  clearpath::SetEStop message;
  message.setEStop(false);
  message.send();
}

void disableMotors()
{
  clearpath::SetEStop message;
  message.setEStop(true);
  message.send();
}

double getWheelDistance(int index)
{
  clearpath::DataPlatform& platform = clearpath::DataPlatform::instance();
  return platform.getWheelDistance(index);
}

void getWheelVelocities(double* velocities)
{
  clearpath::DataPlatform& platform = clearpath::DataPlatform::instance();
  for (int i = 0; i < 4; i++)
  {
    velocities[i] = platform.getWheelVelocity(i);
  }
}

void getWheelAngles(double* angles)
{
  clearpath::DataPlatform& platform = clearpath::DataPlatform::instance();
  for (int i = 0; i < 4; i++)
  {
    angles[i] = platform.getWheelDistance(i);
  }
}

}  // namespace horizon_legacy

[FILENAME: husky_base/include/husky_base/horizon_legacy_wrapper.h]
/**
 *  \file
 *  \brief  C++ interface to Husky's Horizon embedded controller
 *  \author  Clearpath Robotics Inc.
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
#define HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

#include <string>

namespace horizon_legacy
{

/**
 * Clamp value to range [min, max]
 */
double clamp(double value, double min, double max);

/**
 * Attempts to open a serial connection to the Horizon embedded controller
 * @param port The serial port to open
 */
void connect(const std::string& port);

/**
 * Configure the maximum speed and acceleration limits of the platform
 * @param max_speed Maximum speed in m/s
 * @param max_accel Maximum acceleration in m/s^2
 */
void configureLimits(double max_speed, double max_accel);

/**
 * Control the platform's speed
 * @param left_speed Left wheel speed in m/s
 * @param right_speed Right wheel speed in m/s
 */
void controlSpeed(double left_speed, double right_speed);

/**
 * Control the platform's position
 * @param left_distance Left wheel distance in m
 * @param right_distance Right wheel distance in m
 */
void controlDistance(double left_distance, double right_distance);

/**
 * Attempt to read data from the platform
 * @param timeout_sec Timeout in seconds
 */
void getData(double timeout_sec);

/**
 * Enable the platform's motors
 */
void enableMotors();

/**
 * Disable the platform's motors
 */
void disableMotors();

/**
 * Get the distance traveled by the specified wheel
 * @param index Wheel index (0-3)
 * @return Distance traveled in m
 */
double getWheelDistance(int index);

/**
 * Get the velocities of all four wheels
 * @param velocities Array to store the velocities in m/s
 */
void getWheelVelocities(double* velocities);

/**
 * Get the angles of all four wheels
 * @param angles Array to store the angles in rad
 */
void getWheelAngles(double* angles);

}  // namespace horizon_legacy

#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

[FILENAME: husky_base/CMakeLists.txt]
cmake_minimum_required(VERSION 3.8)
project(husky_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(
  include
)

# Horizon library
add_library(horizon_legacy
  src/horizon_legacy/crc.cpp
  src/horizon_legacy/Logger.cpp
  src/horizon_legacy/Message.cpp
  src/horizon_legacy/Message_cmd.cpp
  src/horizon_legacy/Message_data.cpp
  src/horizon_legacy/Message_request.cpp
  src/horizon_legacy/Number.cpp
  src/horizon_legacy/Transport.cpp
)

# Horizon wrapper library
add_library(horizon_legacy_wrapper
  src/horizon_legacy_wrapper.cpp
)
target_link_libraries(horizon_legacy_wrapper horizon_legacy)
ament_target_dependencies(horizon_legacy_wrapper
  rclcpp
)

# Husky hardware interface
add_library(husky_hardware SHARED
  src/husky_hardware.cpp
)
target_link_libraries(husky_hardware
  horizon_legacy_wrapper
)
ament_target_dependencies(husky_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface husky_hardware.xml)

# Install
install(
  TARGETS horizon_legacy horizon_legacy_wrapper husky_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES husky_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  horizon_legacy
  horizon_legacy_wrapper
  husky_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()

[FILENAME: husky_base/husky_hardware.xml]
<library path="husky_hardware">
  <class name="husky_base/HuskyHardware"
         type="husky_base::HuskyHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      ROS2 Control hardware interface for Husky robot.
    </description>
  </class>
</library>