#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "kobuki_msgs/msg/motor_power.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <memory>
#include <string>

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

class KobukiJoystick : public rclcpp::Node
{
public:
  KobukiJoystick()
  : Node("kobuki_joystick"),
    m_input_device("/dev/input/js0"),
    m_scale_linear(0.5f),
    m_scale_angular(1.5f),
    m_fd(-1),
    m_enabled(false)
  {
    m_twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
  }

  bool init()
  {
    this->declare_parameter<std::string>("input_device", m_input_device);
    this->declare_parameter<float>("scale_linear", m_scale_linear);
    this->declare_parameter<float>("scale_angular", m_scale_angular);

    m_input_device = this->get_parameter("input_device").as_string();
    m_scale_linear = static_cast<float>(this->get_parameter("scale_linear").as_double());
    m_scale_angular = static_cast<float>(this->get_parameter("scale_angular").as_double());

    RCLCPP_INFO_STREAM(this->get_logger(),
      "KobukiJoystick : input device [" << m_input_device << "]");

    m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (m_fd == -1)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "KobukiJoystick: Error opening joystick device \""
        << m_input_device
        << "\", is the joystick paired and connected?");
      return false;
    }

    m_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    m_motor_power_publisher = this->create_publisher<std_msgs::msg::UInt8>("motor_power", 1);

    return true;
  }

  void spin()
  {
    rclcpp::Rate loop_rate(60);

    js_event event;
    while (rclcpp::ok())
    {
      if (readEvent(event))
      {
        if (event.type == JS_EVENT_BUTTON && event.number == DS4_BUTTONS::L1)
        {
          if (!m_enabled && event.value == 1)
          {
            enable();
            m_enabled = true;
          }
          else if (m_enabled && event.value == 0)
          {
            disable();
            m_enabled = false;
          }
        }
        else if (event.type == JS_EVENT_AXIS)
        {
          if (event.number == DS4_AXIS::L3_Y)
          {
            m_twist_msg->linear.x = -event.value / 32767.0 * m_scale_linear;
          }
          else if (event.number == DS4_AXIS::R3_X)
          {
            m_twist_msg->angular.z = -event.value / 32767.0 * m_scale_angular;
          }
        }
      }

      if (m_enabled && (m_twist_msg->linear.x != 0 || m_twist_msg->angular.z != 0))
      {
        m_velocity_publisher->publish(*m_twist_msg);
      }

      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }
  }

private:
  bool readEvent(js_event& event)
  {
    if (read(m_fd, &event, sizeof(event)) == -1)
    {
      return false;
    }

    event.type &= ~JS_EVENT_INIT;

    return true;
  }

  void enable()
  {
    kobuki_msgs::msg::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::msg::MotorPower::ON;
    auto msg = std_msgs::msg::UInt8();
    msg.data = power_cmd.state;
    m_motor_power_publisher->publish(msg);
  }

  void disable()
  {
    m_twist_msg->linear.x = 0;
    m_twist_msg->angular.z = 0;
    m_velocity_publisher->publish(*m_twist_msg);

    kobuki_msgs::msg::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::msg::MotorPower::OFF;
    auto msg = std_msgs::msg::UInt8();
    msg.data = power_cmd.state;
    m_motor_power_publisher->publish(msg);
  }

  std::string m_input_device;
  float m_scale_linear;
  float m_scale_angular;
  int m_fd;
  bool m_enabled;
  std::shared_ptr<geometry_msgs::msg::Twist> m_twist_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_motor_power_publisher;
};

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
    RCLCPP_ERROR_STREAM(joystick->get_logger(), "Couldn't initialise KobukiJoystick!");
  }

  RCLCPP_INFO_STREAM(joystick->get_logger(), "Cave Johnson. We're Done Here");

  rclcpp::shutdown();

  return 0;
}