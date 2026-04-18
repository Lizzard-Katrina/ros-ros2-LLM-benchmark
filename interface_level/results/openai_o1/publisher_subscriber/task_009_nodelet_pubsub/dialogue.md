# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
class Plus : public nodelet::Nodelet
{
public:
  Plus()
  : value_(0)
  {}

private:
  virtual void onInit()
  {
    // TODO: initialize node-level communication context
    // retrieve a configurable numeric parameter
    // create a publisher for an output topic and a subscriber
  }

  void callback(/* incoming message */)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    // END OF TODO
  }

  /* publisher handle */
  /* subscriber handle */
  double value_;
};

----------------------------


# LLM Response

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class Plus : public rclcpp::Node
{
public:
  Plus() : Node("plus"), value_(0.0)
  {
    onInit();
  }

private:
  void onInit()
  {
    // TODO: initialize node-level communication context
    // retrieve a configurable numeric parameter
    // create a publisher for an output topic and a subscriber
    value_ = this->declare_parameter<double>("value", 0.0);
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("output_topic", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "input_topic", 10,
      std::bind(&Plus::callback, this, std::placeholders::_1)
    );
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    // END OF TODO
    std_msgs::msg::Float64 output;
    output.data = msg->data + value_;
    publisher_->publish(output);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
  double value_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(Plus)