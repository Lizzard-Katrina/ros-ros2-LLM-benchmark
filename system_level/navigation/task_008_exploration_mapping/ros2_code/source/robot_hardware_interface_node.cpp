#include "task_008_exploration_mapping/robot_hardware_interface.h"

#include <vector>
#include <string>
#include <cstdint>

#ifdef USE_STUB_HW_INTERFACE
  // stubs already included via header
#else
  #include "hardware_interface/system_interface.hpp"
  #include "hardware_interface/handle.hpp"
  #include "hardware_interface/hardware_info.hpp"
  #include "hardware_interface/types/hardware_interface_return_values.hpp"
  #include "hardware_interface/types/hardware_interface_type_values.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "rclcpp_lifecycle/state.hpp"
  #include "pluginlib/class_list_macros.hpp"
  #include "angles/angles.h"
#endif

// ROBOTHardwareInterface inherits from public hardware_interface::SystemInterface

ROBOTHardwareInterface::ROBOTHardwareInterface()
: left_motor_pos(0.0),
  right_motor_pos(0.0),
  left_prev_cmd(0),
  right_prev_cmd(0)
{
}

ROBOTHardwareInterface::~ROBOTHardwareInterface()
{
}

hardware_interface::CallbackReturn ROBOTHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn{hardware_interface::CallbackReturn::SUCCESS})
  {
    return hardware_interface::CallbackReturn{hardware_interface::CallbackReturn::ERROR};
  }

  // Initialize joint names from hardware info
  joint_names_.resize(info_.joints.size());
  joint_position_.resize(info_.joints.size(), 0.0);
  joint_velocity_.resize(info_.joints.size(), 0.0);
  joint_velocity_command_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_names_[i] = info_.joints[i].name;
  }

  // Initialize I2C motor devices (stubbed for non-hardware environments)
  left_motor = I2CDevice(1, 0x08);
  right_motor = I2CDevice(1, 0x09);

  left_motor_pos = 0.0;
  right_motor_pos = 0.0;
  left_prev_cmd = 0;
  right_prev_cmd = 0;

  RCLCPP_INFO(rclcpp::get_logger("ROBOTHardwareInterface"),
    "Hardware interface initialized with %zu joints.", info_.joints.size());

  return hardware_interface::CallbackReturn{hardware_interface::CallbackReturn::SUCCESS};
}

std::vector<hardware_interface::StateInterface>
ROBOTHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ROBOTHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type ROBOTHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t rbuff[1];
  int x;

  left_motor.readBytes(rbuff, 1);
  x = (int8_t)rbuff[0];
  left_motor_pos += angles::from_degrees((double)x);
  joint_position_[0] = left_motor_pos;

  right_motor.readBytes(rbuff, 1);
  x = (int8_t)rbuff[0];
  right_motor_pos += angles::from_degrees((double)x);
  joint_position_[1] = right_motor_pos;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ROBOTHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t wbuff[2];
  int velocity, result;

  velocity = (int)angles::to_degrees(joint_velocity_command_[0]);
  wbuff[0] = velocity;
  wbuff[1] = velocity >> 8;

  if (left_prev_cmd != velocity) {
    result = left_motor.writeData(wbuff, 2);
    (void)result;
    left_prev_cmd = velocity;
  }

  velocity = (int)angles::to_degrees(joint_velocity_command_[1]);
  wbuff[0] = velocity;
  wbuff[1] = velocity >> 8;

  if (right_prev_cmd != velocity) {
    result = right_motor.writeData(wbuff, 2);
    (void)result;
    right_prev_cmd = velocity;
  }

  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(ROBOTHardwareInterface, hardware_interface::SystemInterface)