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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Plus : public rclcpp::Node
{
public:
  Plus()
  : Node("plus"), value_(0)
  {
    // TODO: declare and get a configurable numeric parameter
    this->declare_parameter<double>("value", 0.0);
    this->get_parameter("value", value_);

    // create a publisher for an output topic
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);

    // create a subscriber
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "input_topic", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    auto result = std::make_shared<std_msgs::msg::Int32>();
    result->data = msg->data + static_cast<int>(value_);
    // and publish the processed result
    publisher_->publish(*result);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  double value_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Plus>());
  rclcpp::shutdown();
  return 0;
}
```