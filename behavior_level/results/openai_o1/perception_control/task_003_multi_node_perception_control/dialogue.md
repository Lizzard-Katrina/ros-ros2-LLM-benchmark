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

#include <memory>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_msgs/msg/motor_power.hpp"

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
  R3_X,
  R3_Y,
};

class KobukiJoystick : public rclcpp::Node
{
public:
  KobukiJoystick()
  : Node("kobuki_joystick"),
    m_input_device("/dev/input/js0"),
    m_scale_linear(0.2f),
    m_scale_angular(2.0f),
    m_fd(-1)
  {
  }

  bool init()
  {
    this->declare_parameter<std::string>("input_device", m_input_device);
    this->declare_parameter<float>("scale_linear", m_scale_linear);
    this->declare_parameter<float>("scale_angular", m_scale_angular);

    this->get_parameter("input_device", m_input_device);
    this->get_parameter("scale_linear", m_scale_linear);
    this->get_parameter("scale_angular", m_scale_angular);

    RCLCPP_INFO(this->get_logger(), "KobukiJoystick : input device [%s]", m_input_device.c_str());
    m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (m_fd == -1)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "KobukiJoystick: Error opening joystick device \"%s\", is the joystick paired and connected?",
                   m_input_device.c_str());
      return false;
    }

    m_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    m_motor_power_publisher = this->create_publisher<kobuki_msgs::msg::MotorPower>("motor_power", 1);

    return true;
  }

  void spin()
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
    rclcpp::Rate rate(50);
    bool enabled = false;
    float linear_val = 0.0f;
    float angular_val = 0.0f;

    while (rclcpp::ok())
    {
      js_event event;
      while (readEvent(event))
      {
        if (event.type == JS_EVENT_BUTTON && event.number == DS4_BUTTONS::CIRCLE && event.value == 1)
        {
          enabled = !enabled;
          if (enabled) enable();
          else disable();
        }
        if (event.type == JS_EVENT_AXIS)
        {
          if (event.number == DS4_AXIS::L3_Y)
          {
            // Joystick values range: -32767 to 32767
            linear_val = -event.value / 32767.0f * m_scale_linear;
          }
          else if (event.number == DS4_AXIS::R3_X)
          {
            angular_val = -event.value / 32767.0f * m_scale_angular;
          }
        }
      }

      geometry_msgs::msg::Twist twist;
      if (enabled)
      {
        twist.linear.x = linear_val;
        twist.angular.z = angular_val;
      }
      m_velocity_publisher->publish(twist);

      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
  }

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
    // TODO(task_003):
    // Publish a motor power command that turns the base ON.
    //END OF TODO
    kobuki_msgs::msg::MotorPower power_msg;
    power_msg.state = kobuki_msgs::msg::MotorPower::ON;
    m_motor_power_publisher->publish(power_msg);
  }

  void disable()
  {
    // TODO(task_003):
    // Stop the robot (publish zero velocity),
    // then publish a motor power command that turns the base OFF.
    // namespace kobuki_joystick
    //END OF TODO
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    m_velocity_publisher->publish(stop_msg);

    kobuki_msgs::msg::MotorPower power_msg;
    power_msg.state = kobuki_msgs::msg::MotorPower::OFF;
    m_motor_power_publisher->publish(power_msg);
  }

private:
  std::string m_input_device;
  float m_scale_linear;
  float m_scale_angular;
  int m_fd;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocity_publisher;
  rclcpp::Publisher<kobuki_msgs::msg::MotorPower>::SharedPtr m_motor_power_publisher;
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
    RCLCPP_ERROR(joystick->get_logger(), "Couldn't initialise KobukiJoystick!");
  }

  RCLCPP_INFO(joystick->get_logger(), "Cave Johnson. We're Done Here");
  rclcpp::shutdown();
  return 0;
}