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
#include <nodelet/nodelet.hpp>

namespace ros1_to_ros2_example
{

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
    node_ = std::make_shared<rclcpp::Node>("plus_nodelet");

    // retrieve a configurable numeric parameter
    node_->declare_parameter<double>("value", 0.0);
    node_->get_parameter("value", value_);

    // create a publisher for an output topic and a subscriber
    pub_ = node_->create_publisher<std_msgs::msg::Float64>("output", 10);
    sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "input", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    std_msgs::msg::Float64 output_msg;
    output_msg.data = msg->data + value_;
    pub_->publish(output_msg);
    // END OF TODO
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Node::SharedPtr node_;
  double value_;
};

} // namespace ros1_to_ros2_example

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros1_to_ros2_example::Plus, nodelet::Nodelet)
```