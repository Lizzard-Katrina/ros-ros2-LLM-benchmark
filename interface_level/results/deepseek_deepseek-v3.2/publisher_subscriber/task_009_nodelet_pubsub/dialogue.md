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
    // Initialize node-level communication context
    // Retrieve a configurable numeric parameter
    this->declare_parameter<double>("value", 0.0);
    value_ = this->get_parameter("value").as_double();
    
    // Create a publisher for an output topic and a subscriber
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("output", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "input", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Process the incoming message using the configured parameter
    auto output_msg = std_msgs::msg::Float64();
    output_msg.data = msg->data + value_;
    
    // Publish the processed result
    publisher_->publish(output_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  double value_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Plus>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```