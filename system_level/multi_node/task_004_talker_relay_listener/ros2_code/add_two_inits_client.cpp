#include "rclcpp/rclcpp.hpp"
#include "roscpp_tutorials/TwoInts.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<roscpp_tutorials::TwoInts>("add_two_ints");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting...");
  }
  auto request = std::make_shared<roscpp_tutorials::TwoInts::Request>();
  request->a = std::stoi(argv[1]);
  request->b = std::stoi(argv[2]);
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Result: %d", response->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
  }
  rclcpp::shutdown();
  return 0;
}