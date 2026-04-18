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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <functional>

class Plus : public rclcpp::Node
{
public:
  Plus()
  : rclcpp::Node("plus"), value_(0)
  {
    onInit();
  }

private:
  void onInit()
  {
    // initialize node-level communication context (handled by base class constructor)
    // retrieve a configurable numeric parameter
    this->declare_parameter("value", 0.0);
    this->get_parameter("value", value_);

    // create a publisher for an output topic and a subscriber
    pub_ = this->create_publisher<std_msgs::msg::Float64>("output", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "input", 10, std::bind(&Plus::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // process the incoming message using the configured parameter
    // and publish the processed result
    auto result = std::make_shared<std_msgs::msg::Float64>();
    result->data = msg->data + value_;
    pub_->publish(result);
    // END OF TODO
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  double value_;
};