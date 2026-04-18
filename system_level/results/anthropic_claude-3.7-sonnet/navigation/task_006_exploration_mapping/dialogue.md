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

I'll convert these ROS1 Husky robot files to ROS2, filling in the missing code at TODO locations while maintaining the existing structure and function signatures.

[FILENAME: husky_base/include/husky_base/husky_hardware.h]
/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
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

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <string>
#include <vector>

#include "husky_base/horizon_legacy_wrapper.h"

namespace husky_base
{

class HuskyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HuskyHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string port_;
  double wheel_diameter_, max_accel_, max_speed_;
  
  horizon_legacy::Transport *horizon_legacy_wrapper_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  void resetTravelOffset();
  void registerControlInterfaces();
  double linearToAngular(const double &travel) const;
  double angularToLinear(const double &angle) const;
  void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_HARDWARE_H

[FILENAME: husky_base/src/husky_hardware.cpp]
/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
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

#include "husky_base/husky_hardware.h"
#include <cmath>
#include <stdexcept>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace husky_base
{

hardware_interface::CallbackReturn HuskyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors based on joint count
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Get parameters from URDF
  port_ = info_.hardware_parameters["port"];
  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize the horizon_legacy::Transport for communication with the MCU
  horizon_legacy_wrapper_ = new horizon_legacy::Transport(port_.c_str());
  horizon_legacy_wrapper_->configureLimits(max_speed_, max_accel_);
  resetTravelOffset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export position, velocity, and effort state interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
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

  // Export velocity command interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HuskyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize joint states with current hardware positions
  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  horizon_legacy_wrapper_->setSpeed(0.0, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read from robot hardware
  horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy_wrapper_->getEncoders();
  horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed = horizon_legacy_wrapper_->getDifferentialSpeed();
  horizon_legacy::Channel<clearpath::DataDifferentialOutput>::Ptr diff_out = horizon_legacy_wrapper_->getDifferentialOutput();

  if (enc && speed && diff_out) {
    // Update joint positions and velocities
    double left_pos = linearToAngular(enc->getTravel(horizon_legacy::Channel<clearpath::DataEncoders>::LEFT));
    double right_pos = linearToAngular(enc->getTravel(horizon_legacy::Channel<clearpath::DataEncoders>::RIGHT));
    double left_vel = linearToAngular(speed->getSpeed(horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::LEFT));
    double right_vel = linearToAngular(speed->getSpeed(horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::RIGHT));
    double left_effort = diff_out->getCurrent(horizon_legacy::Channel<clearpath::DataDifferentialOutput>::LEFT);
    double right_effort = diff_out->getCurrent(horizon_legacy::Channel<clearpath::DataDifferentialOutput>::RIGHT);

    // Update joint states
    hw_positions_[0] = left_pos;
    hw_positions_[1] = right_pos;
    hw_velocities_[0] = left_vel;
    hw_velocities_[1] = right_vel;
    hw_efforts_[0] = left_effort;
    hw_efforts_[1] = right_effort;

    return hardware_interface::return_type::OK;
  } else {
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write to robot hardware
  double left_cmd = angularToLinear(hw_commands_[0]);
  double right_cmd = angularToLinear(hw_commands_[1]);

  // Limit the differential speed
  limitDifferentialSpeed(left_cmd, right_cmd);

  // Set the speed on the robot
  horizon_legacy_wrapper_->setSpeed(left_cmd, right_cmd);

  return hardware_interface::return_type::OK;
}

void HuskyHardware::resetTravelOffset()
{
  horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy_wrapper_->getEncoders();
  if (enc) {
    enc->setTravel(horizon_legacy::Channel<clearpath::DataEncoders>::LEFT, 0);
    enc->setTravel(horizon_legacy::Channel<clearpath::DataEncoders>::RIGHT, 0);
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
  if (speed > max_speed_) {
    travel_speed_left = travel_speed_left * max_speed_ / speed;
    travel_speed_right = travel_speed_right * max_speed_ / speed;
  }
}

}  // namespace husky_base

// Register this hardware interface with pluginlib
PLUGINLIB_EXPORT_CLASS(
  husky_base::HuskyHardware,
  hardware_interface::SystemInterface
)

[FILENAME: husky_base/src/horizon_legacy_wrapper.cpp]
/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
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

#include "husky_base/horizon_legacy_wrapper.h"
#include "rclcpp/rclcpp.hpp"

namespace horizon_legacy
{

  Transport::Transport(const char *port)
  {
    RCLCPP_INFO(rclcpp::get_logger("husky_base"), "Initializing Horizon interface on port %s", port);
    int result = clearpath::serial::open(port);
    if (result != 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("husky_base"), "Error initializing Horizon interface: %d", result);
      throw std::runtime_error("Error initializing Horizon interface");
    }
  }

  Transport::~Transport()
  {
    clearpath::serial::close();
  }

  void Transport::configureLimits(double max_speed, double max_accel)
  {
    clearpath::SetMaximumSpeed(max_speed);
    clearpath::SetMaximumAcceleration(max_accel);
  }

  void Transport::setSpeed(double speed_left, double speed_right)
  {
    clearpath::SetDifferentialSpeed(speed_left, speed_right);
  }

  Channel<clearpath::DataEncoders>::Ptr Transport::getEncoders()
  {
    return clearpath::getData<clearpath::DataEncoders>();
  }

  Channel<clearpath::DataDifferentialSpeed>::Ptr Transport::getDifferentialSpeed()
  {
    return clearpath::getData<clearpath::DataDifferentialSpeed>();
  }

  Channel<clearpath::DataDifferentialOutput>::Ptr Transport::getDifferentialOutput()
  {
    return clearpath::getData<clearpath::DataDifferentialOutput>();
  }

  Channel<clearpath::DataPlatformInfo>::Ptr Transport::getPlatformInfo()
  {
    return clearpath::getData<clearpath::DataPlatformInfo>();
  }

  Channel<clearpath::DataPowerSystem>::Ptr Transport::getPowerSystem()
  {
    return clearpath::getData<clearpath::DataPowerSystem>();
  }

  Channel<clearpath::DataSafetySystemStatus>::Ptr Transport::getSafetySystemStatus()
  {
    return clearpath::getData<clearpath::DataSafetySystemStatus>();
  }

}  // namespace horizon_legacy

[FILENAME: husky_base/include/husky_base/horizon_legacy_wrapper.h]
/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
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

#include "horizon_legacy/clearpath.h"
#include "horizon_legacy/transport.h"
#include "horizon_legacy/message_runtime.h"

namespace horizon_legacy
{

  template<typename T>
  class Channel
  {
  public:
    typedef boost::shared_ptr<Channel> Ptr;

    Channel(clearpath::Message::Ptr message) : message_(message)
    {
      data_ = message_->getPayload<T>();
    }

    virtual ~Channel()
    {
    }

    T *get()
    {
      return data_;
    }

    enum
    {
      LEFT = clearpath::LEFT,
      RIGHT = clearpath::RIGHT,
      REAR_LEFT = clearpath::REAR_LEFT,
      REAR_RIGHT = clearpath::REAR_RIGHT
    };

    double getSpeed(int side)
    {
      return data_->getSpeed(static_cast<clearpath::Side>(side));
    }

    double getTravel(int side)
    {
      return data_->getTravel(static_cast<clearpath::Side>(side));
    }

    void setTravel(int side, double val)
    {
      data_->setTravel(static_cast<clearpath::Side>(side), val);
    }

    double getCurrent(int side)
    {
      return data_->getCurrent(static_cast<clearpath::Side>(side));
    }

  private:
    clearpath::Message::Ptr message_;
    T *data_;
  };

  class Transport
  {
  public:
    Transport(const char *port);

    ~Transport();

    void configureLimits(double max_speed, double max_accel);

    void setSpeed(double speed_left, double speed_right);

    Channel<clearpath::DataEncoders>::Ptr getEncoders();

    Channel<clearpath::DataDifferentialSpeed>::Ptr getDifferentialSpeed();

    Channel<clearpath::DataDifferentialOutput>::Ptr getDifferentialOutput();

    Channel<clearpath::DataPlatformInfo>::Ptr getPlatformInfo();

    Channel<clearpath::DataPowerSystem>::Ptr getPowerSystem();

    Channel<clearpath::DataSafetySystemStatus>::Ptr getSafetySystemStatus();
  };

}  // namespace horizon_legacy

#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

[FILENAME: husky_base/CMakeLists.txt]
cmake_minimum_required(VERSION 3.8)
project(husky_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find Horizon Legacy library
find_library(HORIZON_LEGACY_LIB horizon_legacy)

# Add husky hardware interface
add_library(
  ${PROJECT_NAME}
  SHARED
  src/husky_hardware.cpp
  src/horizon_legacy_wrapper.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(
  ${PROJECT_NAME}
  ${HORIZON_LEGACY_LIB}
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

# Export package
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

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
      ROS2 Control hardware interface for Clearpath Husky robot.
    </description>
  </class>
</library>

[FILENAME: husky_base/package.xml]
<?xml version="1.0"?>
<package format="3">
  <name>husky_base</name>
  <version>1.0.0</version>
  <description>Clearpath Husky robot driver</description>

  <author email="pbovbel@clearpathrobotics.com">Paul Bovbel</author>

  <maintainer email="tbaltovski@clearpathrobotics.com">Tony Baltovski</maintainer>

  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>horizon_legacy</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>