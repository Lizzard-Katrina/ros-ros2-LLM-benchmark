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

#include "slamcore_ros1_examples/kobuki_joystick.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_msgs/msg/motor_power.hpp"

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
  m_node->declare_parameter<double>("scale_linear", static_cast<double>(m_scale_linear));
  m_node->declare_parameter<double>("scale_angular", static_cast<double>(m_scale_angular));

  m_node->get_parameter("input_device", m_input_device);
  {
    double scale_linear = static_cast<double>(m_scale_linear);
    double scale_angular = static_cast<double>(m_scale_angular);
    m_node->get_parameter("scale_linear", scale_linear);
    m_node->get_parameter("scale_angular", scale_angular);
    m_scale_linear = static_cast<float>(scale_linear);
    m_scale_angular = static_cast<float>(scale_angular);
  }

  RCLCPP_INFO_STREAM(m_node->get_logger(), "KobukiJoystick : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    RCLCPP_ERROR_STREAM(
      m_node->get_logger(),
      "KobukiJoystick: Error opening joystick device \"" << m_input_device
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
  rclcpp::Rate rate(20.0);
  js_event event;
  geometry_msgs::msg::Twist twist;

  while (rclcpp::ok())
  {
    while (readEvent(event))
    {
      if (event.type == JS_EVENT_BUTTON)
      {
        if (event.number == CROSS && event.value)
        {
          if (m_is_enabled)
          {
            disable();
            m_is_enabled = false;
          }
          else
          {
            enable();
            m_is_enabled = true;
          }
        }
      }
      else if (event.type == JS_EVENT_AXIS)
      {
        if (event.number == L3_Y)
        {
          twist.linear.x = -m_scale_linear * static_cast<float>(event.value) / 32767.0f;
        }
        else if (event.number == R3_X)
        {
          twist.angular.z = -m_scale_angular * static_cast<float>(event.value) / 32767.0f;
        }
      }
    }

    if (m_is_enabled)
    {
      m_velocity_publisher->publish(twist);
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
  kobuki_msgs::msg::MotorPower motor_power;
  motor_power.state = kobuki_msgs::msg::MotorPower::ON;
  m_motor_power_publisher->publish(motor_power);
}

void KobukiJoystick::disable()
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  m_velocity_publisher->publish(twist);

  kobuki_msgs::msg::MotorPower motor_power;
  motor_power.state = kobuki_msgs::msg::MotorPower::OFF;
  m_motor_power_publisher->publish(motor_power);
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
        auto logger = rclcpp::get_logger("kobuki_joystick");
        RCLCPP_ERROR_STREAM(logger, "Couldn't initialise KobukiJoystick!");
    }

    {
      auto logger = rclcpp::get_logger("kobuki_joystick");
      RCLCPP_INFO_STREAM(logger, "Cave Johnson. We're Done Here");
    }

    rclcpp::shutdown();
    return 0;
}