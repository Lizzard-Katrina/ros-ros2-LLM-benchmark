#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class Plus : public rclcpp::Node
{
public:
  explicit Plus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("plus", options), value_(0.0)
  {
    value_ = this->declare_parameter("value", 0.0);
    pub_ = this->create_publisher<std_msgs::msg::Float64>("out", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "in", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std_msgs::msg::Float64 out_msg;
    out_msg.data = msg->data + value_;
    pub_->publish(out_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Plus)