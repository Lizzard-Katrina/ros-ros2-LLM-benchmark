/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// TF
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.hpp>

typedef tf2::TransformException TransformException;
typedef tf2_ros::TransformListener TransformListener;

#include "message_filters/subscriber.hpp"
#include "filters/filter_chain.hpp"

using namespace std::chrono_literals;

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  rclcpp::Node::SharedPtr nh_;

  // Components for tf::MessageFilter
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<TransformListener> tf_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Filter Chain
  std::shared_ptr<filters::FilterChain<sensor_msgs::msg::LaserScan>> filter_chain_;
  bool filter_chain_configured_;

  // Components for publishing
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

  rclcpp::TimerBase::SharedPtr deprecation_timer_;

public:
  // Constructor
  GenericLaserScanFilterNode(rclcpp::Node::SharedPtr nh)
      : nh_(nh),
        filter_chain_configured_(false)
  {
    // Set up TF buffer and listener
    buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_ = std::make_shared<TransformListener>(*buffer_);

    // Configure filter chain
    filter_chain_ = std::make_shared<filters::FilterChain<sensor_msgs::msg::LaserScan>>("sensor_msgs::msg::LaserScan");
    try {
      filter_chain_configured_ = filter_chain_->configure(
          "", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());
    } catch (const std::exception & e) {
      RCLCPP_WARN(nh_->get_logger(), "Filter chain configuration failed: %s. Running in passthrough mode.", e.what());
      filter_chain_configured_ = false;
    }

    if (!filter_chain_configured_) {
      RCLCPP_INFO(nh_->get_logger(), "No filter chain configured. Running in passthrough mode.");
    }

    // Subscribe to scan topic directly
    scan_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&GenericLaserScanFilterNode::callback, this, std::placeholders::_1));

    // Advertise output
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("output", 1000, pub_options);

    deprecation_timer_ = nh_->create_wall_timer(5s, [this]() {
      RCLCPP_WARN(
          nh_->get_logger(),
          "'generic_laser_filter_node' has been deprecated. "
          "Please switch to 'scan_to_scan_filter_chain'.");
    });
  }

  // Callback
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in)
  {
    if (filter_chain_configured_) {
      // Run the filter chain
      filter_chain_->update(*msg_in, msg_);
    } else {
      // Passthrough mode
      msg_ = *msg_in;
    }

    // Publish the output
    output_pub_->publish(msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_filter_node");
  GenericLaserScanFilterNode t(nh);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}