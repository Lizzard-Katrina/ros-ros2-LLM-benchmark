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


#include "scan_to_scan_filter_chain.hpp"


// Constructor
ScanToScanFilterChain::ScanToScanFilterChain(
  const rclcpp::NodeOptions & options,
  const std::string & ns)
: rclcpp::Node("scan_to_scan_filter_chain", ns, options),
  diagnostic_updater_(this),
  tf_(NULL),
  buffer_(this->get_clock()),
  tf_filter_(NULL),
  filter_chain_("sensor_msgs::msg::LaserScan")
{
  // Heartbeat diagnostics
  diagnostic_updater_.add(heartbeat_diagnostics_);
  diagnostic_updater_.setHardwareID("laser_filters");

  // Configure filter chain
  filter_chain_.configure(
    "",
    this->get_node_logging_interface(), this->get_node_parameters_interface());

  rcl_interfaces::msg::ParameterDescriptor read_only_desc;
  read_only_desc.read_only = true;

  // Declare parameters
  #ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->declare_parameter("lazy_subscription", false, read_only_desc);
  #endif
  this->declare_parameter("tf_message_filter_target_frame", "", read_only_desc);
  this->declare_parameter("tf_message_filter_tolerance", 0.03, read_only_desc);
  this->declare_parameter("scan_filtered_history_depth", 1000);

  // Get parameters
  #ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->get_parameter("lazy_subscription", lazy_subscription_);
  #endif
  this->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame_);
  this->get_parameter("tf_message_filter_tolerance", tf_filter_tolerance_);
  this->get_parameter("scan_filtered_history_depth", scan_filtered_history_depth_);

  rclcpp::PublisherOptions pub_options;
#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  if (lazy_subscription_) {
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo & info) {
        if (info.current_count == 0) {
          sub_.unsubscribe();
        } else if (!sub_.getSubscriber()) {
          sub_.subscribe(this, "scan", rmw_qos_profile_sensor_data);
        }
      };
  }
#endif

  output_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_filtered", scan_filtered_history_depth_, pub_options);

#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  if (!lazy_subscription_) {
#endif
    sub_.subscribe(this, "scan", rmw_qos_profile_sensor_data);
#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  }
#endif

  if (tf_message_filter_target_frame_.empty()) {
    sub_.registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
  } else {
    tf_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      sub_, buffer_, tf_message_filter_target_frame_, 50,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(tf_filter_tolerance_));
    tf_filter_->registerCallback(
      std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
  }
}

// Destructor
ScanToScanFilterChain::~ScanToScanFilterChain()
{
  if (tf_filter_) {
    tf_filter_.reset();
  }
  if (tf_) {
    tf_.reset();
  }
}

// Callback
void ScanToScanFilterChain::callback(
  const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg_in)
{
  sensor_msgs::msg::LaserScan msg;
  if (filter_chain_.update(*msg_in, msg)) {
    output_pub_->publish(msg);
    heartbeat_diagnostics_.tick();
  } else {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Filtering the scan from time %f failed.",
      rclcpp::Time(msg_in->header.stamp).seconds());
  }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)