#ifndef TASK_008_EXPLORATION_MAPPING__ROBOT_HARDWARE_INTERFACE_H_
#define TASK_008_EXPLORATION_MAPPING__ROBOT_HARDWARE_INTERFACE_H_

#include <string>
#include <vector>
#include <cstdint>

#ifdef USE_STUB_HW_INTERFACE
  #include "rclcpp/rclcpp.hpp"
  #include "task_008_exploration_mapping/stub_hardware_interface.h"
#else
  #include "hardware_interface/system_interface.hpp"
  #include "hardware_interface/handle.hpp"
  #include "hardware_interface/hardware_info.hpp"
  #include "hardware_interface/types/hardware_interface_return_values.hpp"
  #include "hardware_interface/types/hardware_interface_type_values.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "rclcpp_lifecycle/state.hpp"
  #include "angles/angles.h"
#endif

// Minimal I2C stub for compilation (real hardware not available in Docker)
class I2CDevice {
public:
  I2CDevice() {}
  I2CDevice(int /*bus*/, int /*addr*/) {}
  void readBytes(uint8_t* buf, int len) {
    for (int i = 0; i < len; i++) buf[i] = 0;
  }
  int writeData(uint8_t* /*buf*/, int /*len*/) { return 0; }
};

class ROBOTHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ROBOTHardwareInterface();
  ~ROBOTHardwareInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Joint state storage
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_velocity_command_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Motor position accumulators
  double left_motor_pos;
  double right_motor_pos;

  // Previous commands (for change detection)
  int left_prev_cmd;
  int right_prev_cmd;

  // I2C devices
  I2CDevice left_motor;
  I2CDevice right_motor;
};

#endif  // TASK_008_EXPLORATION_MAPPING__ROBOT_HARDWARE_INTERFACE_H_