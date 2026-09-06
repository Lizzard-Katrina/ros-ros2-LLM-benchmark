#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class CompileCheckNode : public rclcpp::Node
{
public:
  CompileCheckNode() : Node("compile_check_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("compile_check_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CompileCheckNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "reach_study_ros2_ready";
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompileCheckNode>();
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > std::chrono::seconds(3))
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();
  return 0;
}