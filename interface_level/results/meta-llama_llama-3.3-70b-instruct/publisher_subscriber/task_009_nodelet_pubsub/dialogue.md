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
#include <std_msgs/msg/int32.hpp>

class Plus : public rclcpp::Node
{
public:
  Plus()
  : Node("plus_node"), value_(0)
  {
    // TODO: initialize node-level communication context
    this->declare_parameter("numeric_parameter", 1.0);
    // retrieve a configurable numeric parameter
    value_ = this->get_parameter("numeric_parameter").as_double();
    // create a publisher for an output topic and a subscriber
    pub_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("input_topic", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    int32_t result = msg->data + static_cast<int32_t>(value_);
    // and publish the processed result
    std_msgs::msg::Int32 output_msg;
    output_msg.data = result;
    pub_->publish(output_msg);
    // END OF TODO
  }

  /* publisher handle */
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  /* subscriber handle */
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  double value_;
};
```