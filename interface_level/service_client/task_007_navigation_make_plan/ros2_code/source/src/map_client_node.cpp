/*
 * Minimal ROS2 node that demonstrates the service client pattern
 * from the AMCL requestMap() migration.
 *
 * This node:
 *   1. Creates a client for nav_msgs::srv::GetMap
 *   2. Waits for the service
 *   3. Sends a request and processes the response
 */

#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class MapClientNode : public rclcpp::Node
{
public:
  MapClientNode()
  : Node("map_client_node"),
    map_received_(false)
  {
    // Create the service client
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("static_map");

    // Publisher to indicate we got the map (for testing)
    map_info_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("received_map", 1);
  }

  bool requestMap()
  {
    // Wait for the service to become available
    if (!map_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "static_map service not available.");
      return false;
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();

      // Lock mutex before processing (mirrors AMCL pattern)
      std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);

      handleMapMessage(response->map);
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call static_map service.");
      return false;
    }
  }

  bool mapReceived() const { return map_received_; }

  nav_msgs::msg::OccupancyGrid getLastMap() const { return last_map_; }

private:
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
                msg.info.width, msg.info.height, msg.info.resolution);
    last_map_ = msg;
    map_received_ = true;
    map_info_pub_->publish(msg);
  }

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_info_pub_;
  std::recursive_mutex configuration_mutex_;
  bool map_received_;
  nav_msgs::msg::OccupancyGrid last_map_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapClientNode>();

  // Try to request the map
  bool success = node->requestMap();

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Map request succeeded.");
  } else {
    RCLCPP_WARN(node->get_logger(), "Map request failed or service not available.");
  }

  // Spin briefly to allow publishing
  rclcpp::spin_some(node);

  rclcpp::shutdown();
  return 0;
}