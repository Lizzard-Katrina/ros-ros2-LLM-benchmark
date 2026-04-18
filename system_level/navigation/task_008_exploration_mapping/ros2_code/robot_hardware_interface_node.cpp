#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

hardware_interface::CallbackReturn ROBOTHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joint_count = info_.joints.size();
  if (joint_count == 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_position_.assign(joint_count, 0.0);
  joint_velocity_.assign(joint_count, 0.0);
  joint_velocity_command_.assign(joint_count, 0.0);

  left_motor_pos = 0.0;
  right_motor_pos = 0.0;
  left_prev_cmd = std::numeric_limits<int>::min();
  right_prev_cmd = std::numeric_limits<int>::min();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROBOTHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROBOTHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
  }

  return command_interfaces;
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {}

hardware_interface::return_type ROBOTHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  uint8_t rbuff[1];
  int x;

  const double dt = period.seconds();

  left_motor.readBytes(rbuff, 1);
  x = static_cast<int8_t>(rbuff[0]);
  const double left_delta = angles::from_degrees(static_cast<double>(x));
  left_motor_pos += left_delta;
  joint_position_[0] = left_motor_pos;
  joint_velocity_[0] = (dt > 0.0) ? (left_delta / dt) : 0.0;

  right_motor.readBytes(rbuff, 1);
  x = static_cast<int8_t>(rbuff[0]);
  const double right_delta = angles::from_degrees(static_cast<double>(x));
  right_motor_pos += right_delta;
  joint_position_[1] = right_motor_pos;
  joint_velocity_[1] = (dt > 0.0) ? (right_delta / dt) : 0.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ROBOTHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t wbuff[2];
  int velocity;
  int result;

  velocity = static_cast<int>(angles::to_degrees(joint_velocity_command_[0]));
  velocity = std::clamp(velocity, -32768, 32767);
  wbuff[0] = static_cast<uint8_t>(velocity & 0xFF);
  wbuff[1] = static_cast<uint8_t>((velocity >> 8) & 0xFF);

  if (left_prev_cmd != velocity) {
    result = left_motor.writeData(wbuff, 2);
    (void)result;
    left_prev_cmd = velocity;
  }

  velocity = static_cast<int>(angles::to_degrees(joint_velocity_command_[1]));
  velocity = std::clamp(velocity, -32768, 32767);
  wbuff[0] = static_cast<uint8_t>(velocity & 0xFF);
  wbuff[1] = static_cast<uint8_t>((velocity >> 8) & 0xFF);

  if (right_prev_cmd != velocity) {
    result = right_motor.writeData(wbuff, 2);
    (void)result;
    right_prev_cmd = velocity;
  }

  return hardware_interface::return_type::OK;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mobile_robot_hardware_interface");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}