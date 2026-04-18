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

class Plus : public rclcpp::Node
{
public:
  Plus(const rclcpp::NodeOptions & options)
  : Node("plus", options),
    value_(0)
  {
    onInit();
  }

private:
  virtual void onInit()
  {
    // initialize node-level communication context
    this->declare_parameter("value", 0.0);
    value_ = this->get_parameter("value").as_double();
    
    // create a publisher for an output topic and a subscriber
    pub_ = this->create_publisher<std_msgs::msg::Float64>("output", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "input", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    auto out_msg = std_msgs::msg::Float64();
    out_msg.data = msg->data + value_;
    pub_->publish(out_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};
```