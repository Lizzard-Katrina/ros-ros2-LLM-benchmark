#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_type.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/qos.hpp>

class Plus : public rclcpp::Node
{
public:
  Plus() : Node("plus_node"), value_(0)
  {}

private:
  void onInit()
  {
    // Initialize node-level communication context
    this->declare_parameter("numeric_param", 1.0);

    // Retrieve a configurable numeric parameter
    value_ = this->get_parameter("numeric_param").get_value<double>();

    // Create a publisher for an output topic
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("output_topic", 10);

    // Create a subscriber
    subscription_ = this->create_subscription<std_msgs::msg::Float64>("input_topic", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Process the incoming message using the configured parameter
    double result = msg->data + value_;

    // Publish the processed result
    std_msgs::msg::Float64 result_msg;
    result_msg.data = result;
    publisher_->publish(result_msg);
  }

  // Publisher handle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

  // Subscriber handle
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;

  double value_;
};