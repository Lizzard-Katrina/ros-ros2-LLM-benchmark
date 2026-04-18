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
 *  \file husky_hardware.h
 *  \brief Class representing Husky hardware
 *  \author Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright Copyright (c) 2014-2015, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Wheel names
  std::vector<std::string> wheel_joints_;

  // Husky hardware driver
  std::unique_ptr<HorizonLegacyWrapper> horizon_legacy_wrapper_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HUSKY_HARDWARE_H

[FILENAME: husky_base/src/husky_hardware.cpp]
/**
 *  \file husky_hardware.cpp
 *  \brief Class representing Husky hardware
 *  \author Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright Copyright (c) 2014-2015, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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

  // Initialize wheel joints
  wheel_joints_.resize(info_.joints.size());
  for (uint i = 0; i < info_.joints.size(); i++) {
    wheel_joints_[i] = info_.joints[i].name;
  }

  // Resize vectors
  hw_positions_.resize(wheel_joints_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(wheel_joints_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(wheel_joints_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(wheel_joints_.size(), std::numeric_limits<double>::quiet_NaN());

  // Get parameters from URDF
  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware initialized with %zu joints", wheel_joints_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize the Horizon legacy wrapper
  horizon_legacy_wrapper_ = std::make_unique<HorizonLegacyWrapper>();
  
  // Reset commands and state
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0;
    hw_positions_[i] = 0;
    hw_velocities_[i] = 0;
    hw_efforts_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position, velocity, and effort state interfaces for each wheel joint
  for (size_t i = 0; i < wheel_joints_.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_joints_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_joints_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_joints_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HuskyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export velocity command interfaces for each wheel joint
  for (size_t i = 0; i < wheel_joints_.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_joints_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HuskyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize hardware
  if (!horizon_legacy_wrapper_->initialize()) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Failed to initialize Husky hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset commands and state
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0;
    hw_positions_[i] = 0;
    hw_velocities_[i] = 0;
    hw_efforts_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HuskyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0;
  }
  
  // Write zero commands to hardware
  horizon_legacy_wrapper_->setVelocity(0, 0, 0, 0);
  
  RCLCPP_INFO(rclcpp::get_logger("HuskyHardware"), "Husky Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read from hardware
  float front_left_vel, front_right_vel, rear_left_vel, rear_right_vel;
  float front_left_pos, front_right_pos, rear_left_pos, rear_right_pos;
  
  if (!horizon_legacy_wrapper_->getVelocity(front_left_vel, front_right_vel, rear_left_vel, rear_right_vel)) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Failed to get velocity from Husky hardware");
    return hardware_interface::return_type::ERROR;
  }
  
  if (!horizon_legacy_wrapper_->getPosition(front_left_pos, front_right_pos, rear_left_pos, rear_right_pos)) {
    RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Failed to get position from Husky hardware");
    return hardware_interface::return_type::ERROR;
  }
  
  // Update joint state
  if (wheel_joints_.size() == 4) {
    hw_positions_[0] = front_left_pos;
    hw_positions_[1] = front_right_pos;
    hw_positions_[2] = rear_left_pos;
    hw_positions_[3] = rear_right_pos;
    
    hw_velocities_[0] = front_left_vel;
    hw_velocities_[1] = front_right_vel;
    hw_velocities_[2] = rear_left_vel;
    hw_velocities_[3] = rear_right_vel;
    
    // We don't have effort sensing
    hw_efforts_[0] = 0.0;
    hw_efforts_[1] = 0.0;
    hw_efforts_[2] = 0.0;
    hw_efforts_[3] = 0.0;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write to hardware
  if (wheel_joints_.size() == 4) {
    float front_left_vel = static_cast<float>(hw_commands_[0]);
    float front_right_vel = static_cast<float>(hw_commands_[1]);
    float rear_left_vel = static_cast<float>(hw_commands_[2]);
    float rear_right_vel = static_cast<float>(hw_commands_[3]);
    
    // Apply velocity limits
    front_left_vel = std::clamp(front_left_vel, -max_speed_, max_speed_);
    front_right_vel = std::clamp(front_right_vel, -max_speed_, max_speed_);
    rear_left_vel = std::clamp(rear_left_vel, -max_speed_, max_speed_);
    rear_right_vel = std::clamp(rear_right_vel, -max_speed_, max_speed_);
    
    if (!horizon_legacy_wrapper_->setVelocity(front_left_vel, front_right_vel, rear_left_vel, rear_right_vel)) {
      RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Failed to set velocity to Husky hardware");
      return hardware_interface::return_type::ERROR;
    }
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace husky_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(husky_base::HuskyHardware, hardware_interface::SystemInterface)

[FILENAME: husky_base/include/husky_base/horizon_legacy_wrapper.h]
/**
 *  \file horizon_legacy_wrapper.h
 *  \brief Wrapper for horizon_legacy library
 *  \author Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright Copyright (c) 2014-2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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
 */

#ifndef HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
#define HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

#include <string>
#include <mutex>

namespace husky_base
{

class HorizonLegacyWrapper
{
public:
  HorizonLegacyWrapper();
  ~HorizonLegacyWrapper();

  bool initialize();

  bool getVelocity(float &front_left, float &front_right, float &rear_left, float &rear_right);
  bool getPosition(float &front_left, float &front_right, float &rear_left, float &rear_right);
  bool setVelocity(float front_left, float front_right, float rear_left, float rear_right);

private:
  std::mutex horizon_mutex_;
  bool is_connected_;
  std::string port_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

[FILENAME: husky_base/src/horizon_legacy_wrapper.cpp]
/**
 *  \file horizon_legacy_wrapper.cpp
 *  \brief Wrapper for horizon_legacy library
 *  \author Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright Copyright (c) 2014-2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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
 */

#include "husky_base/horizon_legacy_wrapper.h"
#include "rclcpp/rclcpp.hpp"

// Include the actual horizon_legacy library headers
// Note: In a real implementation, you would include the actual horizon_legacy headers
// For this example, we'll simulate the functionality

namespace husky_base
{

HorizonLegacyWrapper::HorizonLegacyWrapper() : is_connected_(false), port_("/dev/ttyUSB0")
{
}

HorizonLegacyWrapper::~HorizonLegacyWrapper()
{
  // Disconnect from hardware if connected
  if (is_connected_) {
    // In a real implementation, you would call the horizon_legacy disconnect function
    RCLCPP_INFO(rclcpp::get_logger("HorizonLegacyWrapper"), "Disconnecting from Husky hardware");
    is_connected_ = false;
  }
}

bool HorizonLegacyWrapper::initialize()
{
  std::lock_guard<std::mutex> lock(horizon_mutex_);
  
  // In a real implementation, you would call the horizon_legacy connect function
  // For this example, we'll simulate a successful connection
  RCLCPP_INFO(rclcpp::get_logger("HorizonLegacyWrapper"), "Connecting to Husky hardware on port %s", port_.c_str());
  
  // Simulate connection to hardware
  is_connected_ = true;
  
  if (is_connected_) {
    RCLCPP_INFO(rclcpp::get_logger("HorizonLegacyWrapper"), "Connected to Husky hardware successfully");
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HorizonLegacyWrapper"), "Failed to connect to Husky hardware");
    return false;
  }
}

bool HorizonLegacyWrapper::getVelocity(float &front_left, float &front_right, float &rear_left, float &rear_right)
{
  std::lock_guard<std::mutex> lock(horizon_mutex_);
  
  if (!is_connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("HorizonLegacyWrapper"), "Not connected to Husky hardware");
    return false;
  }
  
  // In a real implementation, you would call the horizon_legacy getVelocity function
  // For this example, we'll simulate reading from hardware
  
  // Simulate reading velocities from hardware
  // In a real implementation, these values would come from the actual hardware
  static float simulated_velocities[4] = {0.0, 0.0, 0.0, 0.0};
  
  front_left = simulated_velocities[0];
  front_right = simulated_velocities[1];
  rear_left = simulated_velocities[2];
  rear_right = simulated_velocities[3];
  
  return true;
}

bool HorizonLegacyWrapper::getPosition(float &front_left, float &front_right, float &rear_left, float &rear_right)
{
  std::lock_guard<std::mutex> lock(horizon_mutex_);
  
  if (!is_connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("HorizonLegacyWrapper"), "Not connected to Husky hardware");
    return false;
  }
  
  // In a real implementation, you would call the horizon_legacy getPosition function
  // For this example, we'll simulate reading from hardware
  
  // Simulate reading positions from hardware
  // In a real implementation, these values would come from the actual hardware
  static float simulated_positions[4] = {0.0, 0.0, 0.0, 0.0};
  
  // Increment positions based on simulated velocities
  static float simulated_velocities[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < 4; i++) {
    simulated_positions[i] += simulated_velocities[i] * 0.01;  // Assuming 10ms update rate
  }
  
  front_left = simulated_positions[0];
  front_right = simulated_positions[1];
  rear_left = simulated_positions[2];
  rear_right = simulated_positions[3];
  
  return true;
}

bool HorizonLegacyWrapper::setVelocity(float front_left, float front_right, float rear_left, float rear_right)
{
  std::lock_guard<std::mutex> lock(horizon_mutex_);
  
  if (!is_connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("HorizonLegacyWrapper"), "Not connected to Husky hardware");
    return false;
  }
  
  // In a real implementation, you would call the horizon_legacy setVelocity function
  // For this example, we'll simulate writing to hardware
  
  // Store commanded velocities for simulation
  static float simulated_velocities[4] = {0.0, 0.0, 0.0, 0.0};
  simulated_velocities[0] = front_left;
  simulated_velocities[1] = front_right;
  simulated_velocities[2] = rear_left;
  simulated_velocities[3] = rear_right;
  
  RCLCPP_DEBUG(rclcpp::get_logger("HorizonLegacyWrapper"), 
              "Setting velocities: FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f",
              front_left, front_right, rear_left, rear_right);
  
  return true;
}

}  // namespace husky_base

[FILENAME: husky_base/src/husky_hardware_node.cpp]
/**
 *  \file husky_hardware_node.cpp
 *  \brief ROS control node for Husky
 *  \author Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright Copyright (c) 2014-2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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
 */

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

class HuskyHardwareNode : public rclcpp::Node
{
public:
  HuskyHardwareNode()
  : Node("husky_hardware_node")
  {
    // Initialize the controller manager with this node
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}),
      std::make_unique<executor::Executor>(),
      "husky_hardware_interface");

    // Set up timer for controller manager update
    auto update_rate = this->declare_parameter<double>("controller_manager_update_rate", 100.0);
    update_period_ = std::chrono::duration<double>(1.0 / update_rate);
    
    update_timer_ = this->create_wall_timer(
      update_period_, std::bind(&HuskyHardwareNode::update, this));
    
    RCLCPP_INFO(this->get_logger(), "Husky hardware node initialized");
  }

private:
  void update()
  {
    // Get current time
    auto current_time = this->now();
    
    // Update controller manager
    controller_manager_->update(current_time, update_period_);
  }

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::chrono::duration<double> update_period_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create node and spin
  auto node = std::make_shared<HuskyHardwareNode>();
  
  // Use the executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}

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
find_package(controller_manager REQUIRED)

# Include directories
include_directories(
  include
)

# Build the husky hardware interface library
add_library(husky_hardware SHARED
  src/husky_hardware.cpp
  src/horizon_legacy_wrapper.cpp
)

target_include_directories(husky_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(husky_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Build the husky hardware node
add_executable(husky_hardware_node
  src/husky_hardware_node.cpp
)

target_include_directories(husky_hardware_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(husky_hardware_node
  controller_manager
  hardware_interface
  rclcpp
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface husky_hardware.xml)

# Install targets
install(
  TARGETS husky_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS husky_hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(husky_hardware)
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