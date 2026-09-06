#include <rclcpp/rclcpp.hpp>
#include "scan_to_scan_filter_chain.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanToScanFilterChain>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}