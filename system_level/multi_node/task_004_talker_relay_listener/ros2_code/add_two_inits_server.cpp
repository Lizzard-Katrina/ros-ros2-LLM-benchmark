#include "rclcpp/rclcpp.hpp"
#include "roscpp_tutorials/TwoInts.hpp"

bool add(const std::shared_ptr<roscpp_tutorials::TwoInts::Request> request,
         std::shared_ptr<roscpp_tutorials::TwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("add_two_ints_server"), "Incoming request\na: %d, b: %d",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("add_two_ints_server"), "sending back response: [%d]", response->sum);
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_two_ints_server");
  auto service = node->create_service<roscpp_tutorials::TwoInts>("add_two_ints", add);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}