#include <rclcpp/rclcpp.hpp>
#include <task_009_sm_dance_bot_strikes_back/odom_tracker.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cl_move_base_z::odom_tracker::OdomTracker>("odom", "odom");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}