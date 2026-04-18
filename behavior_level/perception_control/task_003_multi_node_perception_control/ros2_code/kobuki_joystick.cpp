#include "slamcore_ros1_examples/kobuki_joystick.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_msgs/msg/motor_power.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace kobuki_joystick
{
namespace
{
rclcpp::Node::SharedPtr g_node;
}  // namespace

enum DS4_BUTTONS
{
  CROSS,
  CIRCLE,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
};

enum DS4_AXIS
{
  L3_X,
  L3_Y,
  //L2, New versions of the driver only.
  R3_X,
  R3_Y,
};

bool KobukiJoystick::init()
{
  g_node = rclcpp::Node::make_shared("kobuki_joystick");

  g_node->declare_parameter<std::string>("input_device", m_input_device);
  g_node->declare_parameter<double>("scale_linear", static_cast<double>(m_scale_linear));
  g_node->declare_parameter<double>("scale_angular", static_cast<double>(m_scale_angular));

  g_node->get_parameter("input_device", m_input_device);

  double scale_linear = static_cast<double>(m_scale_linear);
  double scale_angular = static_cast<double>(m_scale_angular);
  g_node->get_parameter("scale_linear", scale_linear);
  g_node->get_parameter("scale_angular", scale_angular);
  m_scale_linear = static_cast<float>(scale_linear);
  m_scale_angular = static_cast<float>(scale_angular);

  RCLCPP_INFO_STREAM(g_node->get_logger(), "KobukiJoystick : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    RCLCPP_ERROR_STREAM(g_node->get_logger(),
                        "KobukiJoystick: Error opening joystick device \""
                          << m_input_device
                          << "\", is the joystick paired and connected?");
    return false;
  }

  m_velocity_publisher = g_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  m_motor_power_publisher = g_node->create_publisher<kobuki_msgs::msg::MotorPower>("motor_power", 1);

  return true;
}

void KobukiJoystick::spin()
{
  float axes[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool enabled = false;

  rclcpp::Rate rate(50.0);
  while (rclcpp::ok())
  {
    js_event event;
    while (readEvent(event))
    {
      if (event.type == JS_EVENT_BUTTON)
      {
        if (event.number == CROSS && event.value == 1)
        {
          enabled = !enabled;
          if (enabled)
          {
            enable();
          }
          else
          {
            disable();
          }
        }
      }
      else if (event.type == JS_EVENT_AXIS)
      {
        if (event.number <= R3_Y)
        {
          axes[event.number] = static_cast<float>(event.value) / 32767.0f;
        }
      }
    }

    if (enabled)
    {
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = -axes[L3_Y] * m_scale_linear;
      velocity.angular.z = -axes[L3_X] * m_scale_angular;
      m_velocity_publisher->publish(velocity);
    }

    rclcpp::spin_some(g_node);
    rate.sleep();
  }

  if (enabled)
  {
    disable();
  }
}

bool KobukiJoystick::readEvent(js_event& event)
{
  if (read(m_fd, &event, sizeof(event)) == -1)
  {
      return false;
  }

  event.type &= ~JS_EVENT_INIT;

  return true;
}

void KobukiJoystick::enable()
{
  kobuki_msgs::msg::MotorPower power_msg;
  power_msg.state = kobuki_msgs::msg::MotorPower::ON;
  m_motor_power_publisher->publish(power_msg);
}

void KobukiJoystick::disable()
{
  geometry_msgs::msg::Twist stop_msg;
  m_velocity_publisher->publish(stop_msg);

  kobuki_msgs::msg::MotorPower power_msg;
  power_msg.state = kobuki_msgs::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(power_msg);
}
}  // namespace kobuki_joystick

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  kobuki_joystick::KobukiJoystick joystick;
  if (joystick.init())
  {
    joystick.spin();
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("kobuki_joystick"), "Couldn't initialise KobukiJoystick!");
  }

  RCLCPP_INFO(rclcpp::get_logger("kobuki_joystick"), "Cave Johnson. We're Done Here");

  rclcpp::shutdown();
  return 0;
}