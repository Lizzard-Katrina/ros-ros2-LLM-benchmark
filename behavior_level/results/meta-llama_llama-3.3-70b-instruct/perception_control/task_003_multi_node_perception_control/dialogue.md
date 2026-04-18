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
#include "slamcore_ros2_examples/kobuki_joystick.h"

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

class KobukiJoystick : public rclcpp::Node
{
public:
  KobukiJoystick() : Node("kobuki_joystick")
  {
    // TODO(task_003):
    // Initialize node and declare parameters
    this->declare_parameter<std::string>("input_device", "/dev/input/js0");
    this->declare_parameter<float>("scale_linear", 1.0);
    this->declare_parameter<float>("scale_angular", 1.0);

    // Get parameters
    m_input_device = this->get_parameter("input_device").as_string();
    m_scale_linear = this->get_parameter("scale_linear").as_double();
    m_scale_angular = this->get_parameter("scale_angular").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "KobukiJoystick : input device [" << m_input_device << "]");

    // Open joystick device
    m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (m_fd == -1)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "KobukiJoystick: Error opening joystick device \"" << m_input_device << "\", is the joystick paired and connected?");
      rclcpp::shutdown();
    }

    // Create publishers
    m_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    m_motor_power_publisher = this->create_publisher<kobuki_msgs::msg::MotorPower>("motor_power", 1);

    // Create timer
    m_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KobukiJoystick::spin, this));
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
    js_event event;
    if (read(m_fd, &event, sizeof(event)) != -1)
    {
      event.type &= ~JS_EVENT_INIT;

      // Update internal enable/disable state based on a designated button
      if (event.type == JS_EVENT_BUTTON && event.number == CROSS)
      {
        if (event.value == 1)
        {
          enable();
        }
        else
        {
          disable();
        }
      }

      // Map joystick axis values to linear and angular velocity commands (with scaling)
      if (event.type == JS_EVENT_AXIS)
      {
        geometry_msgs::msg::Twist velocity;
        if (event.number == L3_X)
        {
          velocity.linear.x = event.value * m_scale_linear;
        }
        else if (event.number == L3_Y)
        {
          velocity.linear.y = event.value * m_scale_linear;
        }
        else if (event.number == R3_X)
        {
          velocity.angular.z = event.value * m_scale_angular;
        }
        else if (event.number == R3_Y)
        {
          velocity.angular.y = event.value * m_scale_angular;
        }

        // Publish velocity commands only when enabled
        m_velocity_publisher->publish(velocity);
      }
    }
    //END OF TODO
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
    kobuki_msgs::msg::MotorPower motor_power;
    motor_power.state = kobuki_msgs::msg::MotorPower::ON;
    m_motor_power_publisher->publish(motor_power);
    //END OF TODO
  }

  void disable()
  {
    // TODO(task_003):
    // Stop the robot (publish zero velocity),
    // then publish a motor power command that turns the base OFF.
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;
    m_velocity_publisher->publish(velocity);

    kobuki_msgs::msg::MotorPower motor_power;
    motor_power.state = kobuki_msgs::msg::MotorPower::OFF;
    m_motor_power_publisher->publish(motor_power);
    //END OF TODO
  }

private:
  std::string m_input_device;
  float m_scale_linear;
  float m_scale_angular;
  int m_fd;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocity_publisher;
  rclcpp::Publisher<kobuki_msgs::msg::MotorPower>::SharedPtr m_motor_power_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KobukiJoystick>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```