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
#include <example_interfaces/msg/int64.hpp>

class Plus : public rclcpp::Node
{
public:
  Plus()
  : Node("plus")
  {
    // TODO: initialize node-level communication context
    // retrieve a configurable numeric parameter
    this->declare_parameter<double>("value", 0.0);
    this->get_parameter("value", value_);

    // create a publisher for an output topic and a subscriber
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("output", 10);
    subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
      "input", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
    // END OF TODO
  }

private:
  void callback(const example_interfaces::msg::Int64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    auto output = example_interfaces::msg::Int64();
    output.data = msg->data + static_cast<int64_t>(value_);
    publisher_->publish(output);
    // END OF TODO
  }

  /* publisher handle */
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  /* subscriber handle */
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
  double value_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Plus>());
  rclcpp::shutdown();
  return 0;
}
```