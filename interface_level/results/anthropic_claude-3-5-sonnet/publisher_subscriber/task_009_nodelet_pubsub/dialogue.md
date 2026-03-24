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

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

class Plus : public rclcpp::Node
{
public:
  Plus()
  : Node("plus_node"), value_(0)
  {
    // Get parameter
    this->declare_parameter("value", 0.0);
    value_ = this->get_parameter("value").as_double();

    // Create publisher
    pub_ = this->create_publisher<std_msgs::msg::Float64>("out", 10);

    // Create subscriber
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "in", 10,
      std::bind(&Plus::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    auto out = std::make_unique<std_msgs::msg::Float64>();
    out->data = msg->data + value_;
    pub_->publish(*out);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Plus)
```