#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class Plus : public rclcpp::Node
{
public:
  explicit Plus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("plus", options),
    value_(0.0)
  {
    onInit();
  }

private:
  virtual void onInit()
  {
    value_ = this->declare_parameter<double>("value", 0.0);

    pub_ = this->create_publisher<std_msgs::msg::Float64>("output", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "input",
      10,
      std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std_msgs::msg::Float64 out;
    out.data = msg->data + value_;
    pub_->publish(out);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(Plus)