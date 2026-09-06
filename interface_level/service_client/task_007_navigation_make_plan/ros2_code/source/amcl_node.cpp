/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>

// Signal handling
#include <signal.h>

#include "rclcpp/rclcpp.hpp"

// Messages that I need
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "std_srvs/srv/empty.hpp"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode();

  void savePoseToServer();

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool sent_first_transform_;
  tf2::Transform latest_tf_;
  bool latest_tf_valid_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;

  bool use_map_topic_;
  bool first_map_only_;

  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

  bool first_map_received_;

  std::recursive_mutex configuration_mutex_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

  void requestMap();
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg);
};

AmclNode::AmclNode()
: Node("amcl"),
  sent_first_transform_(false),
  latest_tf_valid_(false),
  first_map_received_(false)
{
  // Declare parameters
  this->declare_parameter<bool>("use_map_topic", false);
  this->declare_parameter<bool>("first_map_only", false);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("global_frame_id", "map");

  this->get_parameter("use_map_topic", use_map_topic_);
  this->get_parameter("first_map_only", first_map_only_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("global_frame_id", global_frame_id_);

  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // Create the service client for GetMap
  map_client_ = this->create_client<nav_msgs::srv::GetMap>("static_map");

  if (!use_map_topic_) {
    requestMap();
  }
}

AmclNode::~AmclNode()
{
}

void
AmclNode::requestMap()
{
  // Wait for the service to become available, retrying periodically
  while (!map_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the static_map service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for static_map service to become available...");
  }

  // Create and send the request
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto future = map_client_->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Received map from static_map service.");

    // Lock the configuration mutex before processing the map
    std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);
    handleMapMessage(response->map);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call static_map service.");
  }
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg)
{
  RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
              msg.info.width,
              msg.info.height,
              msg.info.resolution);
  first_map_received_ = true;
}

void
AmclNode::savePoseToServer()
{
  // Placeholder
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}