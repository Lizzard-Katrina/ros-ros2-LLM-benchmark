#include "slamcore_ros1_examples/kobuki_joystick.hpp"

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
  m_node->declare_parameter<float>("scale_linear", m_scale_linear);
  m_node->declare_parameter<float>("scale_angular", m_scale_angular);

  m_node->get_parameter("input_device", m_input_device);
  m_node->get_parameter("scale_linear", m_scale_linear);
  m_node->get_parameter("scale_angular", m_scale_angular);

  RCLCPP_INFO_STREAM(m_node->get_logger(), "KobukiJoystick : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    RCLCPP_ERROR_STREAM(m_node->get_logger(), "KobukiJoystick: Error opening joystick device \""
                     << m_input_device
                     << "\", is the joystick paired and connected?");
    return false;
  }

  m_velocity_publisher = m_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  m_motor_power_publisher =
    m_node->create_publisher<kobuki_msgs::msg::MotorPower>("motor_power", 1);

  return true;
}

void KobukiJoystick::spin()
{
  rclcpp::Rate rate(50);
  js_event event;
  geometry_msgs::msg::Twist cmd;
  bool enabled = false;

  while (rclcpp::ok())
  {
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
        if (event.number == L3_Y)
        {
          cmd.linear.x = -event.value / 32767.0 * m_scale_linear;
        }
        else if (event.number == L3_X)
        {
          cmd.angular.z = -event.value / 32767.0 * m_scale_angular;
        }
      }
    }

    if (enabled)
    {
      m_velocity_publisher->publish(cmd);
    }

    rclcpp::spin_some(m_node);
    rate.sleep();
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
  kobuki_msgs::msg::MotorPower power_cmd;
  power_cmd.state = kobuki_msgs::msg::MotorPower::ON;
  m_motor_power_publisher->publish(power_cmd);
}

void KobukiJoystick::disable()
{
  geometry_msgs::msg::Twist cmd;
  m_velocity_publisher->publish(cmd);

  kobuki_msgs::msg::MotorPower power_cmd;
  power_cmd.state = kobuki_msgs::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(power_cmd);
}

} // namespace kobuki_joystick

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
        std::cerr << "Couldn't initialise KobukiJoystick!" << std::endl;
    }

    std::cout << "Cave Johnson. We're Done Here" << std::endl;

    rclcpp::shutdown();
    return 0;
}