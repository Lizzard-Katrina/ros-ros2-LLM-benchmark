#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("babbler");
  auto publisher = node->create_publisher<std_msgs::msg::String>("babble", 10);
  auto timer = node->create_wall_timer(std::chrono::milliseconds(100),
    [publisher, node]() mutable {
      auto msg = std_msgs::msg::String();
      static int count = 0;
      std::stringstream ss;
      ss << "hello world: " << count;
      msg.data = ss.str();
      RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.data.c_str());
      publisher->publish(msg);
      count++;
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}