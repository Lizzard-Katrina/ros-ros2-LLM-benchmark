#ifndef TASK_003_KOBUKI_JOYSTICK_H
#define TASK_003_KOBUKI_JOYSTICK_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <linux/joystick.h>
#include <string>
#include <memory>

namespace kobuki_joystick
{

struct MotorPower
{
  static constexpr uint8_t ON = 1;
  static constexpr uint8_t OFF = 0;
  uint8_t state = OFF;
};

class KobukiJoystick : public rclcpp::Node
{
public:
  KobukiJoystick();
  ~KobukiJoystick();

  bool init();
  void spin_loop();

private:
  bool readEvent(js_event& event);
  void enable();
  void disable();

  std::string m_input_device{"/dev/input/js0"};
  float m_scale_linear{0.5f};
  float m_scale_angular{1.0f};
  int m_fd{-1};
  bool m_enabled{false};

  geometry_msgs::msg::Twist m_twist_msg;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_motor_power_publisher;
};

}  // namespace kobuki_joystick

#endif