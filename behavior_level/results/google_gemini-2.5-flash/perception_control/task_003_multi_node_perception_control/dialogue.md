# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include "slamcore_ros1_examples/kobuki_joystick.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/MotorPower.h"

#include <fcntl.h>
#include <unistd.h>

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
  ros::NodeHandle nh("~");

  nh.param<std::string>("input_device", m_input_device, m_input_device);
  nh.param<float>("scale_linear", m_scale_linear, m_scale_linear);
  nh.param<float>("scale_angular", m_scale_angular, m_scale_angular);

  ROS_INFO_STREAM("KobukiJoystick : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    ROS_ERROR_STREAM("KobukiJoystick: Error opening joystick device \""
                     << m_input_device
                     << "\", is the joystick paired and connected?");
    return false;
  }

  m_velocity_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  m_motor_power_publisher =
    nh.advertise<kobuki_msgs::MotorPower>("motor_power", 1);

  return true;
}

void KobukiJoystick::spin()
{
// TODO(task_003):
  // Implement the main control loop:
  // - read joystick events
  // - update internal enable/disable state based on a designated button
  // - map joystick axis values to linear and angular velocity commands (with scaling)
  // - publish velocity commands only when enabled
  //
  // The loop should run at a fixed rate while ROS is running
  //END OF TODO
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
  // TODO(task_003):
  // Publish a motor power command that turns the base ON.
  //END OF TODO
}

void KobukiJoystick::disable()
{
// TODO(task_003):
  // Stop the robot (publish zero velocity),
  // then publish a motor power command that turns the base OFF.
  // namespace kobuki_joystick
  //END OF TODO
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kobuki_joystick");

    kobuki_joystick::KobukiJoystick joystick;
    if (joystick.init())
    {
        joystick.spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise KobukiJoystick!");
    }

    ROS_INFO_STREAM("Cave Johnson. We're Done Here");

    return 0;
}

----------------------------


# LLM Response

```cpp
#include "slamcore_ros1_examples/kobuki_joystick.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/motor_power.hpp"

#include <fcntl.h>
#include <unistd.h>

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
  m_node->get_parameter("input_device", m_input_device);
  m_node->declare_parameter<float>("scale_linear", m_scale_linear);
  m_node->get_parameter("scale_linear", m_scale_linear);
  m_node->declare_parameter<float>("scale_angular", m_scale_angular);
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
    m_node->create_publisher<kobuki_ros_interfaces::msg::MotorPower>("motor_power", 1);

  return true;
}

void KobukiJoystick::spin()
{
  rclcpp::Rate loop_rate(10); // 10 Hz

  while (rclcpp::ok())
  {
    js_event event;
    while (readEvent(event))
    {
      if (event.type == JS_EVENT_BUTTON)
      {
        if (event.number == CROSS)
        {
          if (event.value == 1) // Button pressed
          {
            m_enabled = !m_enabled;
            if (m_enabled)
            {
              enable();
              RCLCPP_INFO(m_node->get_logger(), "KobukiJoystick: Enabled");
            }
            else
            {
              disable();
              RCLCPP_INFO(m_node->get_logger(), "KobukiJoystick: Disabled");
            }
          }
        }
      }
      else if (event.type == JS_EVENT_AXIS)
      {
        if (event.number == L3_Y)
        {
          m_linear_velocity = -static_cast<float>(event.value) / 32767.0f * m_scale_linear;
        }
        else if (event.number == L3_X)
        {
          m_angular_velocity = -static_cast<float>(event.value) / 32767.0f * m_scale_angular;
        }
      }
    }

    if (m_enabled)
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = m_linear_velocity;
      twist_msg.angular.z = m_angular_velocity;
      m_velocity_publisher->publish(twist_msg);
    }

    rclcpp::spin_some(m_node);
    loop_rate.sleep();
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
  auto motor_power_msg = kobuki_ros_interfaces::msg::MotorPower();
  motor_power_msg.state = kobuki_ros_interfaces::msg::MotorPower::ON;
  m_motor_power_publisher->publish(motor_power_msg);
}

void KobukiJoystick::disable()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  m_velocity_publisher->publish(twist_msg);

  auto motor_power_msg = kobuki_ros_interfaces::msg::MotorPower();
  motor_power_msg.state = kobuki_ros_interfaces::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(motor_power_msg);
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
    RCLCPP_ERROR(rclcpp::get_logger("kobuki_joystick"), "Couldn't initialise KobukiJoystick!");
  }

  RCLCPP_INFO(rclcpp::get_logger("kobuki_joystick"), "Cave Johnson. We're Done Here");

  rclcpp::shutdown();
  return 0;
}
```