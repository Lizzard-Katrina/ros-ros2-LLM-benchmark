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
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class Plus : public rclcpp::Node
{
public:
  explicit Plus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("plus", options), value_(0.0)
  {
    onInit();
  }

private:
  void onInit()
  {
    // retrieve a configurable numeric parameter
    value_ = this->declare_parameter<double>("value", 0.0);

    // create a publisher for an output topic and a subscriber
    pub_ = this->create_publisher<std_msgs::msg::Float64>("out", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "in", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    std_msgs::msg::Float64 out_msg;
    out_msg.data = msg->data + value_;
    pub_->publish(out_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};
```