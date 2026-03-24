#include "slamcore_ros1_examples/kobuki_joystick.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_msgs/msg/motor_power.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

namespace kobuki_joystick
{
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
  m_node = rclcpp::Node::make_shared("kobuki_joystick");

  m_node->declare_parameter<std::string>("input_device", m_input_device);
  m_node->declare_parameter<double>("scale_linear", m_scale_linear);
  m_node->declare_parameter<double>("scale_angular", m_scale_angular);

  m_node->get_parameter("input_device", m_input_device);
  m_node->get_parameter("scale_linear", m_scale_linear);
  m_node->get_parameter("scale_angular", m_scale_angular);

  RCLCPP_INFO(m_node->get_logger(), "KobukiJoystick : input device [%s]", m_input_device.c_str());
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    RCLCPP_ERROR(m_node->get_logger(), "KobukiJoystick: Error opening joystick device \"%s\", is the joystick paired and connected?", m_input_device.c_str());
    return false;
  }

  m_velocity_publisher = m_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  m_motor_power_publisher = m_node->create_publisher<kobuki_msgs::msg::MotorPower>("motor_power", 1);

  m_enabled = false;

  return true;
}

void KobukiJoystick::spin()
{
  rclcpp::Rate rate(50);
  js_event event;
  int16_t axis_values[4] = {0, 0, 0, 0};

  while (rclcpp::ok())
  {
    while (readEvent(event))
    {
      if (event.type == JS_EVENT_BUTTON)
      {
        if (event.number == DS4_BUTTONS::CROSS && event.value == 1)
        {
          if (!m_enabled)
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
        if (event.number >= 0 && event.number < 4)
        {
          axis_values[event.number] = event.value;
        }
      }
    }

    if (m_enabled)
    {
      geometry_msgs::msg::Twist twist_msg;
      // L3_X axis controls angular velocity (left/right)
      // L3_Y axis controls linear velocity (forward/backward)
      twist_msg.linear.x = -axis_values[DS4_AXIS::L3_Y] / 32767.0 * m_scale_linear;
      twist_msg.angular.z = axis_values[DS4_AXIS::L3_X] / 32767.0 * m_scale_angular;
      m_velocity_publisher->publish(twist_msg);
    }

    rclcpp::spin_some(m_node);
    rate.sleep();
  }
}

bool KobukiJoystick::readEvent(js_event& event)
{
  ssize_t bytes = read(m_fd, &event, sizeof(event));
  if (bytes == -1)
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
  m_enabled = true;
}

void KobukiJoystick::disable()
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  m_velocity_publisher->publish(twist_msg);

  kobuki_msgs::msg::MotorPower power_msg;
  power_msg.state = kobuki_msgs::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(power_msg);
  m_enabled = false;
}
}  // namespace kobuki_joystick

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto joystick = std::make_shared<kobuki_joystick::KobukiJoystick>();
  if (joystick->init())
  {
    joystick->spin();
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't initialise KobukiJoystick!");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cave Johnson. We're Done Here");

  rclcpp::shutdown();
  return 0;
}