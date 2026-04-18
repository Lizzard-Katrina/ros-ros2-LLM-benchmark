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

I'll convert these ROS1 files to ROS2, filling in the missing code at TODO locations while maintaining the existing structure and functionality.

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
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include "husky_base/horizon_legacy_wrapper.h"

namespace husky_base
{

/**
 * Class representing Husky hardware, allows for ros_control to modify internal state via joint interfaces
 */
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
  * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
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

  // ROS Parameters
  double wheel_diameter_, max_accel_, max_speed_;

  // Hardware handles
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Horizon interface
  horizon_legacy::Transport horizon_transport_;
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
#include "husky_base/husky_hardware.h"
#include "rclcpp/rclcpp.hpp"

namespace husky_base
{

HuskyHardware::HuskyHardware() :
  port_("/dev/prolific"),
  polling_timeout_(10.0)
{
}

hardware_interface::CallbackReturn HuskyHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage for hardware state
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Get parameters from URDF
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check if joint is a command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HuskyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HuskyHardware"),
        "Joint '%s' has %s command interface. Expected %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if joint has state interface
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HuskyHardware"),
        "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Get parameters from hardware_info
  for (const auto & parameter : info_.hardware_parameters)
  {
    if (parameter.first == "port")
    {
      port_ = parameter.second;
    }
    if (parameter.first == "wheel_diameter")
    {
      wheel_diameter_ = std::stod(parameter.second);
    }
    if (parameter.first == "max_accel")
    {
      max_accel_ = std::stod(parameter.second);
    }
    if (parameter.first == "max_speed")
    {
      max_speed_ = std::stod(parameter.second);
    }
    if (parameter.first == "polling_timeout")
    {
      polling_timeout_ = std::stod(parameter.second);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Connect to the horizon_legacy::Transport
  horizon_transport_.connect(port_);
  
  // Initialize the joints
  resetTravelOffset();
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export joint state interfaces
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HuskyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export joint command interfaces
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::CallbackReturn HuskyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands to zero
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read from robot hardware
  horizon_legacy::Channel<clearpath::DataDifferentialSpeed> speed;
  horizon_legacy::Channel<clearpath::DataDifferentialControl> control;
  horizon_legacy::Channel<clearpath::DataPower> power;
  horizon_legacy::Channel<clearpath::DataSafetyStatus> safety;
  horizon_legacy::Channel<clearpath::DataEncoders> encoders;

  // Populate data from the robot
  try
  {
    horizon_transport_.receive(speed);
    horizon_transport_.receive(control);
    horizon_transport_.receive(power);
    horizon_transport_.receive(safety);
    horizon_transport_.receive(encoders);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Error receiving from robot: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Convert encoder counts to radians
  double left_wheel_angle = linearToAngular(encoders.getTravel(0));
  double right_wheel_angle = linearToAngular(encoders.getTravel(1));

  // Update joint information
  hw_positions_[0] = left_wheel_angle;
  hw_positions_[1] = right_wheel_angle;
  hw_velocities_[0] = linearToAngular(speed.getLeftSpeed());
  hw_velocities_[1] = linearToAngular(speed.getRightSpeed());

  // Update rear wheel positions and velocities based on front wheels
  hw_positions_[2] = hw_positions_[0];
  hw_positions_[3] = hw_positions_[1];
  hw_velocities_[2] = hw_velocities_[0];
  hw_velocities_[3] = hw_velocities_[1];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write to robot hardware
  double left_speed_cmd = angularToLinear(hw_commands_[0]);
  double right_speed_cmd = angularToLinear(hw_commands_[1]);

  // Limit speeds
  limitDifferentialSpeed(left_speed_cmd, right_speed_cmd);

  // Set the speed
  try
  {
    horizon_legacy::controlSpeed(left_speed_cmd, right_speed_cmd, max_accel_, &horizon_transport_);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Error sending commands to robot: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void HuskyHardware::resetTravelOffset()
{
  horizon_legacy::Channel<clearpath::DataEncoders> encoders;
  
  // Retrieve encoder data from the robot
  try
  {
    horizon_transport_.receive(encoders);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Error receiving encoder data: %s", e.what());
    return;
  }

  // Initialize position offsets using current encoder values
  for (int i = 0; i < 4; i++)
  {
    joints_[i].position_offset = linearToAngular(encoders.getTravel(i % 2));
  }
}

double HuskyHardware::linearToAngular(const double &travel) const
{
  return travel / wheel_diameter_ * 2;
}

double HuskyHardware::angularToLinear(const double &angle) const
{
  return angle * wheel_diameter_ / 2;
}

void HuskyHardware::limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right)
{
  double speed = std::max(std::abs(travel_speed_left), std::abs(travel_speed_right));
  if (speed > max_speed_)
  {
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
 *  \brief  C++ interface to Clearpath Robotics Horizon protocol.
 *  \author  Ryan Gariepy <rgariepy@clearpathrobotics.com>
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
#include "husky_base/horizon_legacy_wrapper.h"

namespace horizon_legacy
{

Transport::Transport() : configured_(false), fd_(-1)
{
}

Transport::~Transport()
{
  if (configured_)
  {
    close(fd_);
  }
}

void Transport::configure(const char *port)
{
  fd_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd_ < 0)
  {
    throw std::runtime_error("Failed to open device");
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0)
  {
    throw std::runtime_error("Failed to get terminal attributes");
  }

  options.c_cflag = CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  if (tcsetattr(fd_, TCSANOW, &options) < 0)
  {
    throw std::runtime_error("Failed to set terminal attributes");
  }

  configured_ = true;
}

void Transport::connect(const std::string& port)
{
  configure(port.c_str());
}

void Transport::send(const Message& command)
{
  if (!configured_)
  {
    throw std::runtime_error("Transport not configured");
  }

  int bytes_written = write(fd_, command.data(), command.size());
  if (bytes_written < 0)
  {
    throw std::runtime_error("Failed to write to device");
  }
  if (bytes_written != static_cast<int>(command.size()))
  {
    throw std::runtime_error("Incomplete write to device");
  }
}

void Transport::receive(Message& response)
{
  if (!configured_)
  {
    throw std::runtime_error("Transport not configured");
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;  // 500ms

  int select_result = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);
  if (select_result < 0)
  {
    throw std::runtime_error("Error in select()");
  }
  else if (select_result == 0)
  {
    throw std::runtime_error("Timeout waiting for data");
  }

  unsigned char buffer[1024];
  int bytes_read = read(fd_, buffer, sizeof(buffer));
  if (bytes_read < 0)
  {
    throw std::runtime_error("Failed to read from device");
  }

  response.parse(buffer, bytes_read);
}

void controlSpeed(double left_speed, double right_speed, double accel_limit, Transport *transport)
{
  clearpath::SetDifferentialSpeed command(left_speed, right_speed, accel_limit);
  transport->send(command);
}

}  // namespace horizon_legacy

[FILENAME: husky_base/include/husky_base/horizon_legacy_wrapper.h]
/**
 *  \file
 *  \brief  C++ interface to Clearpath Robotics Horizon protocol.
 *  \author  Ryan Gariepy <rgariepy@clearpathrobotics.com>
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
#include <stdexcept>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace clearpath
{
  // Forward declarations of message classes
  class Message;
  class SetDifferentialSpeed;
  class DataDifferentialSpeed;
  class DataDifferentialControl;
  class DataPower;
  class DataSafetyStatus;
  class DataEncoders;
}

namespace horizon_legacy
{

class Message
{
public:
  virtual ~Message() {}
  virtual const unsigned char* data() const = 0;
  virtual size_t size() const = 0;
  virtual void parse(const unsigned char* buffer, size_t length) = 0;
};

template<typename T>
class Channel : public Message
{
public:
  Channel() : msg_() {}
  
  const unsigned char* data() const override
  {
    return reinterpret_cast<const unsigned char*>(&msg_);
  }
  
  size_t size() const override
  {
    return sizeof(T);
  }
  
  void parse(const unsigned char* buffer, size_t length) override
  {
    if (length >= sizeof(T))
    {
      msg_ = *reinterpret_cast<const T*>(buffer);
    }
    else
    {
      throw std::runtime_error("Buffer too small for message type");
    }
  }
  
  // Accessor methods for specific message types
  double getLeftSpeed() const { return msg_.getLeftSpeed(); }
  double getRightSpeed() const { return msg_.getRightSpeed(); }
  double getTravel(int index) const { return msg_.getTravel(index); }
  
private:
  T msg_;
};

class Transport
{
public:
  Transport();
  ~Transport();

  void configure(const char *port);
  void connect(const std::string& port);
  void send(const Message& command);
  void receive(Message& response);

private:
  bool configured_;
  int fd_;
};

void controlSpeed(double left_speed, double right_speed, double accel_limit, Transport *transport);

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

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/husky_hardware.cpp
  src/horizon_legacy_wrapper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface husky_hardware_plugin.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()

[FILENAME: husky_base/husky_hardware_plugin.xml]
<library path="husky_base">
  <class name="husky_base/HuskyHardware"
         type="husky_base::HuskyHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      ROS2 Control hardware interface for Husky robot.
    </description>
  </class>
</library>

[FILENAME: husky_base/package.xml]
<?xml version="1.0"?>
<package format="3">
  <name>husky_base</name>
  <version>1.0.0</version>
  <description>Clearpath Husky robot driver</description>

  <author email="rgariepy@clearpathrobotics.com">Ryan Gariepy</author>
  <author email="paul@bovbel.com">Paul Bovbel</author>
  <author email="tbaltovski@clearpathrobotics.com">Tony Baltovski</author>

  <maintainer email="tbaltovski@clearpathrobotics.com">Tony Baltovski</maintainer>

  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>