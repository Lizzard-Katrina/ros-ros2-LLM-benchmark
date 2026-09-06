#ifndef TURTLESIM__QOS_HPP_
#define TURTLESIM__QOS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace turtlesim
{

inline rclcpp::QoS topic_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10));
}

}  // namespace turtlesim

#endif  // TURTLESIM__QOS_HPP_