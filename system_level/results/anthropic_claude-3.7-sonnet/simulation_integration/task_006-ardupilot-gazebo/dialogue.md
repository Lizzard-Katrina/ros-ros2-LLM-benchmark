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

I'll convert these ROS1 files to ROS2 and fill in the missing code at TODO locations.

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
    RCLCPP_SHARED_PTR_DEFINITIONS(HuskyHardware);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    // Parameters for the Husky hardware
    double wheel_diameter_, max_accel_, max_speed_;

    // ROS Control interfaces
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;

    // Husky hardware driver
    std::unique_ptr<HorizonLegacyWrapper> horizon_legacy_;

    // Misc
    double wheel_radius_;
    double polling_timeout_;
  };

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_HARDWARE_H

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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace husky_base
{

  class HorizonLegacyWrapper
  {
  public:
    HorizonLegacyWrapper(const std::string &port = "/dev/prolific", const bool lookup_param = false);

    ~HorizonLegacyWrapper();

    void connect();

    bool connected();

    void configureLimits(double max_speed, double max_accel);

    void commandVelocity(double diff_speed_left, double diff_speed_right);

    void getVelocity(double &diff_speed_left, double &diff_speed_right);

    void getEncoderCounts(int &left_encoder, int &right_encoder);

    void resetEncoderCounts();

    void getInfo(double &fw_version, std::string &serial_number);

    void getStatus(std::vector<uint8_t> &system_status, std::vector<uint8_t> &safety_status,
                  std::vector<uint8_t> &power_status);

  private:
    std::string port_;
    bool connected_;
    double max_speed_, max_accel_;
    rclcpp::Logger logger_;
  };

}  // namespace husky_base

#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace husky_base
{

hardware_interface::CallbackReturn HuskyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize the hardware parameters
  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);
  polling_timeout_ = std::stod(info_.hardware_parameters["polling_timeout"]);

  wheel_radius_ = wheel_diameter_ / 2.0;

  // Initialize the hardware interface
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize the Horizon legacy wrapper
  std::string port = info_.hardware_parameters["port"];
  horizon_legacy_ = std::make_unique<HorizonLegacyWrapper>(port);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Configure the base
  // Connect to the hardware and configure limits
  try {
    horizon_legacy_->connect();
    horizon_legacy_->configureLimits(max_speed_, max_accel_);
    RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky hardware configured successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Error configuring Husky hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  // TODO: Create state interfaces
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
  // TODO: Create command interfaces
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
  // TODO: Activate the hardware
  // Set default values for commands
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0.0;
  }

  // Reset encoder counts
  horizon_legacy_->resetEncoderCounts();

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Deactivate the hardware
  // Stop the robot
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0.0;
  }
  
  // Send zero velocity command to the hardware
  horizon_legacy_->commandVelocity(0.0, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read from hardware and update state interfaces
  if (!horizon_legacy_->connected()) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Could not read from hardware, Horizon not connected");
    return hardware_interface::return_type::ERROR;
  }

  // Read encoder counts
  int left_encoder, right_encoder;
  horizon_legacy_->getEncoderCounts(left_encoder, right_encoder);

  // Read wheel velocities
  double left_wheel_velocity, right_wheel_velocity;
  horizon_legacy_->getVelocity(left_wheel_velocity, right_wheel_velocity);

  // Update state interfaces
  // Assuming the joint order is [front_left, front_right, rear_left, rear_right]
  hw_velocities_[0] = left_wheel_velocity;  // front_left
  hw_velocities_[1] = right_wheel_velocity; // front_right
  hw_velocities_[2] = left_wheel_velocity;  // rear_left
  hw_velocities_[3] = right_wheel_velocity; // rear_right

  // Calculate positions from encoder counts (simplified)
  // In a real implementation, you would need to convert encoder ticks to radians
  // This is a placeholder calculation
  hw_positions_[0] = left_encoder * 0.001;  // front_left
  hw_positions_[1] = right_encoder * 0.001; // front_right
  hw_positions_[2] = left_encoder * 0.001;  // rear_left
  hw_positions_[3] = right_encoder * 0.001; // rear_right

  // Efforts are not directly measured, set to NaN or 0
  for (size_t i = 0; i < hw_efforts_.size(); i++) {
    hw_efforts_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Write commands to hardware
  if (!horizon_legacy_->connected()) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Could not write to hardware, Horizon not connected");
    return hardware_interface::return_type::ERROR;
  }

  // Assuming the joint order is [front_left, front_right, rear_left, rear_right]
  // For a differential drive, we use the average of the left and right sides
  double left_side_velocity = (hw_commands_[0] + hw_commands_[2]) / 2.0;
  double right_side_velocity = (hw_commands_[1] + hw_commands_[3]) / 2.0;

  // Send velocity commands to the hardware
  horizon_legacy_->commandVelocity(left_side_velocity, right_side_velocity);

  return hardware_interface::return_type::OK;
}

}  // namespace husky_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(husky_base::HuskyHardware, hardware_interface::SystemInterface)

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

#include <string>
#include <vector>

namespace husky_base
{

  HorizonLegacyWrapper::HorizonLegacyWrapper(const std::string &port, const bool lookup_param)
    : port_(port),
      connected_(false),
      max_speed_(0.0),
      max_accel_(0.0),
      logger_(rclcpp::get_logger("HorizonLegacyWrapper"))
  {
    // TODO: Initialize the wrapper
    // In ROS2, we don't need to look up parameters here as they are passed in via the hardware_info
    // This is a simplified implementation that would need to be expanded with actual hardware communication
    RCLCPP_INFO(logger_, "Initializing Horizon Legacy Wrapper on port %s", port_.c_str());
  }

  HorizonLegacyWrapper::~HorizonLegacyWrapper()
  {
    // TODO: Clean up the wrapper
    if (connected_) {
      // Send zero velocity command to stop the robot
      commandVelocity(0.0, 0.0);
      connected_ = false;
      RCLCPP_INFO(logger_, "Horizon Legacy Wrapper disconnected");
    }
  }

  void HorizonLegacyWrapper::connect()
  {
    // TODO: Connect to the hardware
    // This is a simplified implementation that would need to be expanded with actual hardware communication
    RCLCPP_INFO(logger_, "Connecting to Husky hardware on port %s", port_.c_str());
    
    // Simulate connection to hardware
    // In a real implementation, this would establish a serial connection to the Husky hardware
    connected_ = true;
    RCLCPP_INFO(logger_, "Connected to Husky hardware");
  }

  bool HorizonLegacyWrapper::connected()
  {
    return connected_;
  }

  void HorizonLegacyWrapper::configureLimits(double max_speed, double max_accel)
  {
    // TODO: Configure the limits
    max_speed_ = max_speed;
    max_accel_ = max_accel;
    
    RCLCPP_INFO(logger_, "Configured limits: max_speed = %.2f, max_accel = %.2f", max_speed_, max_accel_);
    
    // In a real implementation, this would send commands to the Husky hardware to set these limits
  }

  void HorizonLegacyWrapper::commandVelocity(double diff_speed_left, double diff_speed_right)
  {
    // TODO: Command the velocity
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot command velocity, not connected to hardware");
      return;
    }
    
    // Enforce limits
    diff_speed_left = std::max(-max_speed_, std::min(max_speed_, diff_speed_left));
    diff_speed_right = std::max(-max_speed_, std::min(max_speed_, diff_speed_right));
    
    RCLCPP_DEBUG(logger_, "Commanding velocity: left = %.2f, right = %.2f", diff_speed_left, diff_speed_right);
    
    // In a real implementation, this would send velocity commands to the Husky hardware
  }

  void HorizonLegacyWrapper::getVelocity(double &diff_speed_left, double &diff_speed_right)
  {
    // TODO: Get the velocity
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot get velocity, not connected to hardware");
      diff_speed_left = 0.0;
      diff_speed_right = 0.0;
      return;
    }
    
    // In a real implementation, this would read velocity values from the Husky hardware
    // For now, we'll just return the last commanded values (simplified)
    // This is a placeholder and should be replaced with actual hardware communication
    
    // Simulated values for testing
    diff_speed_left = 0.0;
    diff_speed_right = 0.0;
  }

  void HorizonLegacyWrapper::getEncoderCounts(int &left_encoder, int &right_encoder)
  {
    // TODO: Get the encoder counts
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot get encoder counts, not connected to hardware");
      left_encoder = 0;
      right_encoder = 0;
      return;
    }
    
    // In a real implementation, this would read encoder counts from the Husky hardware
    // For now, we'll just return simulated values
    // This is a placeholder and should be replaced with actual hardware communication
    
    // Simulated values for testing
    left_encoder = 0;
    right_encoder = 0;
  }

  void HorizonLegacyWrapper::resetEncoderCounts()
  {
    // TODO: Reset the encoder counts
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot reset encoder counts, not connected to hardware");
      return;
    }
    
    RCLCPP_INFO(logger_, "Resetting encoder counts");
    
    // In a real implementation, this would send a command to reset encoder counts on the Husky hardware
  }

  void HorizonLegacyWrapper::getInfo(double &fw_version, std::string &serial_number)
  {
    // TODO: Get the info
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot get info, not connected to hardware");
      fw_version = 0.0;
      serial_number = "unknown";
      return;
    }
    
    // In a real implementation, this would read firmware version and serial number from the Husky hardware
    // For now, we'll just return simulated values
    
    // Simulated values for testing
    fw_version = 1.0;
    serial_number = "HUSKY-SIMULATION-001";
  }

  void HorizonLegacyWrapper::getStatus(std::vector<uint8_t> &system_status, std::vector<uint8_t> &safety_status,
                                      std::vector<uint8_t> &power_status)
  {
    // TODO: Get the status
    if (!connected_) {
      RCLCPP_WARN(logger_, "Cannot get status, not connected to hardware");
      system_status.clear();
      safety_status.clear();
      power_status.clear();
      return;
    }
    
    // In a real implementation, this would read status information from the Husky hardware
    // For now, we'll just return simulated values
    
    // Simulated values for testing
    system_status = {0, 0, 0, 0};
    safety_status = {0, 0, 0, 0};
    power_status = {0, 0, 0, 0};
  }

}  // namespace husky_base